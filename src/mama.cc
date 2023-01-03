#include "mama.hpp"

#include "graph/graph.hpp"
#include "s2/s2point.h"
#include "state.pb.h"
#include <filesystem>
#include <fstream>
#include <iostream>

namespace mama {

struct EmissionCost {
public:
  explicit EmissionCost(double sigma_z = 4.07)
      : inv_double_sq_sigma_z_(1.f / (sigma_z * sigma_z * 2.f)) {}
  double operator()(const Projection &projection) const {
    return -inv_double_sq_sigma_z_ * std::pow(projection.distance_m, 2.0);
  }

private:
  double inv_double_sq_sigma_z_;
};

struct TransitionCost {
public:
  explicit TransitionCost(double haversine_distance, double beta = 5.0)
      : haversine_distance_(haversine_distance), inversed_beta_(beta) {}
  std::optional<double> operator()(double path_distance) const {
    return -std::abs(path_distance - haversine_distance_) * inversed_beta_;
  }

private:
  double haversine_distance_;
  double inversed_beta_;
};

MapMatcher::MapMatcher(std::shared_ptr<Graph> graph)
    : graph_(std::move(graph)) {}

namespace {

std::vector<std::vector<double>>
BuildPathMatrix(const std::vector<PointOnGraph> &from_candidates,
                const std::vector<PointOnGraph> &to_candidates,
                std::shared_ptr<Graph> graph) {
  std::vector<std::vector<double>> path_matrix;
  path_matrix.reserve(from_candidates.size());
  for (const auto &from_candidate : from_candidates) {
    auto path_distances = graph->PathDistance(from_candidate, to_candidates, {});
    path_matrix.emplace_back(std::move(path_distances));
  }
  return path_matrix;
}

} // namespace

Location MapMatcher::Update(const Location &input_location,
                            state::State &state) {
  auto location = input_location;

  auto candidates = graph_->Project(location.coordinate, 100);
  if (candidates.empty()) {
    return location;
  }

  EmissionCost emission_cost_computer{};

  if (state.hmm_states().empty()) {
    // initialize
    state.clear_hmm_states();
    for (const auto &candidate : candidates) {
      auto hmm_state = state.add_hmm_states();
      hmm_state->set_sequence_cost(emission_cost_computer(candidate));
    }
  } else {
    assert(state.has_previous_location());
    if (!state.has_previous_location()) {
      // something wrong happened, reset
      state = {};
      return Update(input_location, state);
    }
    auto previous_location = ConvertProtoToLocation(state.previous_location());
    TransitionCost transition_cost_computer{
        input_location.coordinate.Distance(previous_location.coordinate)};

    std::vector<PointOnGraph>
        candidate_points;
    candidate_points.reserve(candidates.size());
    for (const auto &candidate : candidates) {
      candidate_points.push_back(candidate.point_on_graph);
    }

    std::vector<PointOnGraph> previous_candidate_points;
    previous_candidate_points.reserve(state.hmm_states().size());
    for (const auto &hmm_state : state.hmm_states()) {
      PointOnGraph point_on_graph;
      point_on_graph.edge_id = {
          hmm_state.point_on_graph().edge_id().tile_id(),
          hmm_state.point_on_graph().edge_id().edge_index()};
      point_on_graph.offset = hmm_state.point_on_graph().offset();

      previous_candidate_points.push_back(point_on_graph);
    }

    std::vector<double> previous_candidate_sequence_costs;
    previous_candidate_sequence_costs.reserve(state.hmm_states().size());
    for (const auto &hmm_state : state.hmm_states()) {
      previous_candidate_sequence_costs.push_back(hmm_state.sequence_cost());
    }

    state.clear_hmm_states();

    auto path_matrix =
        BuildPathMatrix(previous_candidate_points, candidate_points, graph_);

    for (size_t candidate_index = 0; candidate_index < candidate_points.size();
         ++candidate_index) {
      const auto &candidate = candidates[candidate_index];
      auto emission_cost = emission_cost_computer(candidate);

      double max_cost = -std::numeric_limits<double>::infinity();
      for (size_t previous_candidate_index = 0;
           previous_candidate_index < previous_candidate_points.size();
           ++previous_candidate_index) {
        auto transition_cost = transition_cost_computer(
            path_matrix[previous_candidate_index][candidate_index]);
        if (!transition_cost) {
          continue;
        }
        auto cost =
            previous_candidate_sequence_costs[previous_candidate_index] +
            *transition_cost + emission_cost;
        if (cost > max_cost) {
          max_cost = cost;
        }
      }
      if (max_cost != -std::numeric_limits<double>::infinity()) {
        auto hmm_state = state.add_hmm_states();
        hmm_state->set_sequence_cost(max_cost);
      }
    }

    if (state.hmm_states().empty()) {
      // HMM is broken, reset
      state = {};
      return Update(input_location, state);
    }
  }

  // TODO: guarantee that we always properly update it
  *state.mutable_previous_location() =
      ConvertLocationToProto<state::Location>(input_location);

  auto result = BuildResult(location, candidates, state);
  *state.mutable_previous_matched_location() =
      ConvertLocationToProto<state::Location>(result);
  return result;
}

Location MapMatcher::BuildResult(const Location &location,
                                 const std::vector<Projection> &candidates,
                                 const state::State &state) {
  if (candidates.empty()) {
    return location;
  }
  assert(candidates.size() == state.hmm_states_size());

  size_t best_index = 0;
  for (size_t i = 1; i < candidates.size(); ++i) {
    if (state.hmm_states(i).sequence_cost() >
        state.hmm_states(best_index).sequence_cost()) {
      best_index = i;
    }
  }

  auto result = location;
  result.coordinate = candidates[best_index].coordinate;
  result.bearing = candidates[best_index].bearing_deg;
  return result;
}

Location MapMatchingController::Update(const Location &location,
                                       std::string &state) {
  state::State state_proto;
  state_proto.ParseFromString(state);
  auto result = Update(location, state_proto);
  state_proto.SerializeToString(&state);
  return result;
}

Location MapMatchingController::Update(Location location, state::State &state) {
  if (state.has_previous_location()) {
    auto previous_location = ConvertProtoToLocation(state.previous_location());
    if (previous_location.timestamp >= location.timestamp) {
      assert(state.has_previous_matched_location());
      if (!state.has_previous_matched_location()) {
        // something went wrong if it happened, so reset the state
        state = {};
        return Update(location, state);
      }
      return ConvertProtoToLocation(state.previous_matched_location());
    }

    if (!location.bearing) {
      auto bearing =
          previous_location.coordinate.BearingTo(location.coordinate);
      location.bearing = bearing;
    }
    if (!location.speed) {
      auto timestamp_diff = location.timestamp.ToSeconds() -
                            previous_location.timestamp.ToSeconds();
      auto distance =
          location.coordinate.Distance(previous_location.coordinate);
      location.speed = distance / timestamp_diff;
    }
  }

  constexpr auto kMinSpeedMps = 1.0;
  if (location.speed && *location.speed < kMinSpeedMps &&
      state.has_previous_matched_location()) {
    *state.mutable_previous_location() =
        ConvertLocationToProto<state::Location>(location);
    return ConvertProtoToLocation(state.previous_matched_location());
  }

  auto result = map_matcher_->Update(location, state);

  *state.mutable_previous_location() =
      ConvertLocationToProto<state::Location>(location);

  return result;
}

} // namespace mama
