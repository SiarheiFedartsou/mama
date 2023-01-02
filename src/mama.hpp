#pragma once

#include "base/coordinate.hpp"
#include "graph/graph.hpp"
#include <optional>
namespace mama {

namespace state {
class State;
}

struct Timestamp {
  int64_t seconds;
  int32_t nanos;

  double ToSeconds() const { return seconds + nanos * 1e-9; }

  // TODO: can we clean it up
  bool operator==(const Timestamp &other) const {
    return seconds == other.seconds && nanos == other.nanos;
  }

  bool operator<(const Timestamp &other) const {
    return seconds < other.seconds ||
           (seconds == other.seconds && nanos < other.nanos);
  }

  bool operator>(const Timestamp &other) const {
    return seconds > other.seconds ||
           (seconds == other.seconds && nanos > other.nanos);
  }

  bool operator<=(const Timestamp &other) const {
    return seconds < other.seconds ||
           (seconds == other.seconds && nanos <= other.nanos);
  }

  bool operator>=(const Timestamp &other) const {
    return seconds > other.seconds ||
           (seconds == other.seconds && nanos >= other.nanos);
  }
};

struct Location {
  Timestamp timestamp;
  Coordinate coordinate;
  std::optional<double> bearing = 0.0;
  std::optional<double> speed = 0.0;
  std::optional<double> horizontal_accuracy = 0.0;
};

template <typename T> T ConvertLocationToProto(const Location &location) {
  T location_proto;
  location_proto.mutable_timestamp()->set_seconds(location.timestamp.seconds);
  location_proto.mutable_timestamp()->set_nanos(location.timestamp.nanos);
  location_proto.set_longitude(location.coordinate.lng());
  location_proto.set_latitude(location.coordinate.lat());
  if (location.bearing) {
    location_proto.mutable_bearing()->set_value(*location.bearing);
  }
  if (location.speed) {
    location_proto.mutable_speed()->set_value(*location.speed);
  }
  return location_proto;
}

template <typename T> Location ConvertProtoToLocation(const T &location_proto) {
  Location location;
  location.timestamp.seconds = location_proto.timestamp().seconds();
  location.timestamp.nanos = location_proto.timestamp().nanos();
  location.coordinate = {location_proto.longitude(), location_proto.latitude()};
  if (location_proto.has_bearing()) {
    location.bearing = location_proto.bearing().value();
  }
  if (location_proto.has_speed()) {
    location.speed = location_proto.speed().value();
  }
  return location;
}

class MapMatcher {
public:
  explicit MapMatcher(std::shared_ptr<Graph> graph);

  // the same as `state::State` overload, but automatically serializes the state
  Location Update(const Location &location, std::string &state);
  Location Update(const Location &location, state::State &state);
  std::shared_ptr<Graph> graph_;

private:
  std::optional<Location> FilterAndImpute(const Location &location,
                                          const state::State &state);
  Location BuildResult(const Location &location,
                       const std::vector<Projection> &candidates,
                       const state::State &state);
};
} // namespace mama