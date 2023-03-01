#include "mama.hpp"

#include "graph/graph.hpp"
#include "base/angle.hpp"

#include "s2/s2point.h"
#include "state.pb.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <base/log.hpp>
namespace mama {

using PathMatrix = std::vector<std::vector<double>>;



struct EmissionCost {
public:
  explicit EmissionCost(double distance_weight = 0.03,
                        double bearing_weight = 1.0)
      : distance_weight_(distance_weight), bearing_weight_(bearing_weight) {}
  double operator()(const Projection &projection,
                    const Location &location) const {
    const auto distance_cost =
        -distance_weight_ * std::pow(projection.distance_m, 2.0);
    auto bearing_cost = 0.0;

    if (location.bearing) {
      const auto bearing_diff =
          AngleDiff(projection.bearing_deg, *location.bearing) / 180.0;
      bearing_cost = -bearing_weight_ * std::pow(bearing_diff, 2);
    }

    return distance_cost + bearing_cost;
  }

private:
  double distance_weight_;
  double bearing_weight_;
};

struct TransitionCost {
public:
  explicit TransitionCost(double haversine_distance,
                          double distance_diff_weight = 0.2)
      : haversine_distance_(haversine_distance),
        distance_diff_weight_(distance_diff_weight) {}
  std::optional<double> operator()(double path_distance) const {
    return -distance_diff_weight_ *
           std::abs(path_distance - haversine_distance_);
  }

private:
  double haversine_distance_;
  double distance_diff_weight_;
};

MapMatcher::MapMatcher(std::shared_ptr<Graph> graph)
    : graph_(std::move(graph)) {}

namespace {

PathMatrix BuildPathMatrix(const std::vector<PointOnGraph> &from_candidates,
                           const std::vector<PointOnGraph> &to_candidates,
                           std::shared_ptr<Graph> graph) {
  std::vector<std::vector<double>> path_matrix;
  path_matrix.reserve(from_candidates.size());
  for (const auto &from_candidate : from_candidates) {
    auto path_distances =
        graph->PathDistance(from_candidate, to_candidates, {});
    path_matrix.emplace_back(std::move(path_distances));
  }
  return path_matrix;
}

} // namespace

Location MapMatcher::Update(const Location &input_location) {
  auto candidates = graph_->Project(input_location.coordinate, 25);
  if (candidates.empty()) {
    return input_location;
  }

  EmissionCost emission_cost_computer{};

  if (last_states_.empty()) {
    // initialize
    for (const auto &candidate : candidates) {
      HMMState state;
      state.point_on_graph = candidate.point_on_graph;
      state.sequence_cost = emission_cost_computer(candidate, input_location);
      last_states_.push_back(state);
    }
  } else {
    assert(previous_location_);
    if (!previous_location_) {
      // something wrong happened, reset
      last_states_.clear();
      return Update(input_location);
    }
    TransitionCost transition_cost_computer{
        input_location.coordinate.Distance(previous_location_->coordinate)};

    std::vector<PointOnGraph> candidate_points;
    candidate_points.reserve(candidates.size());
    for (const auto &candidate : candidates) {
      candidate_points.push_back(candidate.point_on_graph);
    }

    std::vector<PointOnGraph> previous_candidate_points;
    previous_candidate_points.reserve(last_states_.size());
    for (const auto &hmm_state : last_states_) {
      previous_candidate_points.push_back(hmm_state.point_on_graph);
    }

    std::vector<double> previous_candidate_sequence_costs;
    previous_candidate_sequence_costs.reserve(last_states_.size());
    for (const auto &hmm_state : last_states_) {
      previous_candidate_sequence_costs.push_back(hmm_state.sequence_cost);
    }

    last_states_.clear();
    auto path_matrix =
        BuildPathMatrix(previous_candidate_points, candidate_points, graph_);
      
    for (size_t candidate_index = 0; candidate_index < candidate_points.size();
         ++candidate_index) {
      const auto &candidate = candidates[candidate_index];
      auto emission_cost = emission_cost_computer(candidate, input_location);

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
      // TODO: we can avoid saving `-std::numeric_limits<double>::infinity()`
      // values, but need to somehow fix `BuildResult` in this case
      HMMState hmm_state;
      hmm_state.point_on_graph = candidate.point_on_graph;
      hmm_state.sequence_cost = max_cost;
      last_states_.push_back(hmm_state);
    }

    if (last_states_.empty()) {
      // HMM is broken, reset
      previous_location_ = {};
      return Update(input_location);
    }
  }

  previous_location_ = input_location;

  auto result = BuildResult(input_location, candidates);
  return result;
}

Location MapMatcher::BuildResult(const Location &location,
                                 const std::vector<Projection> &candidates) {
  if (candidates.empty()) {
    return location;
  }
  assert(candidates.size() == last_states_.size());

  size_t best_index = 0;
  for (size_t i = 1; i < candidates.size(); ++i) {
    if (last_states_[i].sequence_cost >
        last_states_[best_index].sequence_cost) {
      best_index = i;
    }
  }

  auto result = location;
  result.coordinate = candidates[best_index].coordinate;
  result.bearing = candidates[best_index].bearing_deg;
  return result;
}

void MapMatcher::RestoreState(const state::State &pbf_state) {
  last_states_.clear();
  for (const auto &pbf_hmm_state : pbf_state.hmm_states()) {
    HMMState state;
    state.point_on_graph = {
        {pbf_hmm_state.point_on_graph().edge_id().tile_id(),
         pbf_hmm_state.point_on_graph().edge_id().edge_index()},
        pbf_hmm_state.point_on_graph().offset()};
    state.sequence_cost = pbf_hmm_state.sequence_cost();
    last_states_.emplace_back(state);
  }
  previous_location_ = ConvertProtoToLocation(pbf_state.previous_location());
}

void MapMatcher::SaveState(state::State &pbf_state) {
  pbf_state.clear_hmm_states();
  for (const auto &state : last_states_) {
    auto pbf_hmm_state = pbf_state.add_hmm_states();
    pbf_hmm_state->mutable_point_on_graph()->mutable_edge_id()->set_tile_id(
        state.point_on_graph.edge_id.tile_id);
    pbf_hmm_state->mutable_point_on_graph()->mutable_edge_id()->set_edge_index(
        state.point_on_graph.edge_id.edge_index);
    pbf_hmm_state->mutable_point_on_graph()->set_offset(
        state.point_on_graph.offset);
    pbf_hmm_state->set_sequence_cost(state.sequence_cost);
  }
  assert(previous_location_);
  *pbf_state.mutable_previous_location() =
      ConvertLocationToProto<state::Location>(*previous_location_);
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
  MAMA_TRACE("Update()");
  RestoreState(state);
  auto result = Update(location);
  SaveState(state);
  return result;
}

Location MapMatchingController::Update(Location location) {
  if (previous_location_) {
    if (previous_location_->timestamp >= location.timestamp) {
      assert(previous_matched_location_);
      // TODO: we probably should return error to client in this case
      return *previous_matched_location_;
    }

    if (!location.bearing) {
      auto bearing =
          previous_location_->coordinate.BearingTo(location.coordinate);
      location.bearing = bearing;
    }
    if (!location.speed) {
      auto timestamp_diff = location.timestamp.ToSeconds() -
                            previous_location_->timestamp.ToSeconds();
      auto distance =
          location.coordinate.Distance(previous_location_->coordinate);
      location.speed = distance / timestamp_diff;
    }
  }

  constexpr auto kMinSpeedMps = 1.0;
  if (location.speed && *location.speed < kMinSpeedMps &&
      previous_matched_location_) {
    previous_location_ = location;
    return *previous_matched_location_;
  }
  auto result = map_matcher_->Update(location);
  previous_location_ = location;
  previous_matched_location_ = result;
  return result;
}

void MapMatchingController::RestoreState(const state::State &state) {
  map_matcher_->RestoreState(state);
  if (state.has_previous_location() && state.has_previous_matched_location()) {
    previous_location_ = ConvertProtoToLocation(state.previous_location());
    previous_matched_location_ =
        ConvertProtoToLocation(state.previous_matched_location());
  } else {
    previous_location_ = {};
    previous_matched_location_ = {};
  }
}

void MapMatchingController::SaveState(state::State &state) {
  map_matcher_->SaveState(state);
  assert(previous_location_);
  assert(previous_matched_location_);
  *state.mutable_previous_location() =
      ConvertLocationToProto<state::Location>(*previous_location_);
  *state.mutable_previous_matched_location() =
      ConvertLocationToProto<state::Location>(*previous_matched_location_);
}

} // namespace mama
