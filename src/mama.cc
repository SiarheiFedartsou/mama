#include "mama.hpp"

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
  double operator()(const Location &from_location, const Location &to_location,
                    float path_distance) const {
    return 0.0;
  }
};

MapMatcher::MapMatcher(std::shared_ptr<Graph> graph)
    : graph_(std::move(graph)) {}

std::optional<Location> MapMatcher::FilterAndImpute(const Location &location,
                                                    const state::State &state) {
  if (!state.has_previous_location()) {
    return location;
  }
  auto previous_location = ConvertProtoToLocation(state.previous_location());

  if (state.has_previous_location() && location.timestamp <= previous_location.timestamp) {
    return std::nullopt;
  }
  Location result = location;
  if (!location.bearing) {
    auto bearing = previous_location.coordinate.BearingTo(location.coordinate);
    result.bearing = bearing;
  }
  if (!location.speed) {
    // TODO: `timestamp_diff` can be negative
    auto timestamp_diff = location.timestamp.ToSeconds() -
                          previous_location.timestamp.ToSeconds();
    auto distance = location.coordinate.Distance(previous_location.coordinate);
    std::cerr << "distance: " << distance << " timestamp: " << timestamp_diff << std::endl;

    if (distance > 0) {
      std::cerr << "BINGO\n";
    }
    result.speed = distance / timestamp_diff;
  }
  return result;
}

Location MapMatcher::Update(const Location &input_location,
                                           state::State &state) {                                       
  // auto filtered_location = FilterAndImpute(input_location, state);
  // if (!filtered_location) {
  //   if (!state.has_previous_matched_location()) {
  //     // something went wrong, so reset state
  //     assert(false);
  //     state = {};
  //     return Update(input_location, state);      
  //   }
  //   return ConvertProtoToLocation(state.previous_matched_location());
  // }

  auto location = input_location;

 // std::cerr << "location: " << location.speed.value_or(-1) << std::endl;

  auto candidates = graph_->Project(location.coordinate, 100);
  if (candidates.empty()) {
    return location;
  }

  EmissionCost emission_cost_computer{};

  // if (state.hmm_states().empty()) {
  state.clear_hmm_states();
  for (const auto &candidate : candidates) {
    auto hmm_state = state.add_hmm_states();
    hmm_state->set_sequence_cost(emission_cost_computer(candidate));
  }
  //   // initialize
  // } else {
  //   for (const auto &candidate : candidates) {
  //     auto emission_cost = emission_cost_computer(candidate);
  //     auto transition_cost = 0.f;

  //     std::vector<PointOnGraph> candidate_points;
  //     candidate_points.reserve(candidates.size());
  //     for (const auto &candidate : candidates) {
  //       candidate_points.push_back(candidate.point_on_graph);
  //     }

  //     for (const auto &hmm_state : state.hmm_states()) {
  //       PointOnGraph from_point;
  //       from_point.edge_id = {
  //           hmm_state.point_on_graph().edge_id().tile_id(),
  //           hmm_state.point_on_graph().edge_id().edge_index()};
  //       from_point.offset = hmm_state.point_on_graph().offset();

  //       auto path_distances =
  //           graph_->PathDistance(from_point, candidate_points);

  //       // auto hmm_state = state.add_hmm_states();
  //       // hmm_state->set_sequence_cost(emission_cost);
  //     }
  //   }
  //   // update
  // }

  // TODO: guarantee that we always properly update it
  *state.mutable_previous_location() = ConvertLocationToProto<state::Location>(input_location);

  auto result = BuildResult(location, candidates, state);
  *state.mutable_previous_matched_location() = ConvertLocationToProto<state::Location>(result);
  return result;  
}

Location MapMatcher::Update(const Location &location,
                                           std::string &state) {
  state::State state_proto;
  state_proto.ParseFromString(state);
  auto result = Update(location, state_proto);
  state_proto.SerializeToString(&state);
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

} // namespace mama
