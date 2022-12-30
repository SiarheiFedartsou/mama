#pragma once

#include "graph.hpp"
#include <optional>
namespace mama {

namespace state {
class State;
}

struct Location {
  double timestamp;
  Coordinate coordinate;
  std::optional<double> bearing = 0.0;
  std::optional<double> speed = 0.0;
  std::optional<double> horizontal_accuracy = 0.0;
};
class MapMatcher {
public:
  explicit MapMatcher(std::shared_ptr<Graph> graph);

  // the same as `state::State` overload, but automatically serializes the state
  Location Update(const Location &location, std::string &state);
  Location Update(const Location &location, state::State &state);
  std::shared_ptr<Graph> graph_;

private:
   Location BuildResult(const Location& location, const std::vector<Projection>& candidates, const state::State& state);
};
} // namespace mama