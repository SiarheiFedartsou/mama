
#pragma once

#include "base/coordinate.hpp"

#include <osmium/handler.hpp>

#include <unordered_map>

namespace mama {
namespace tilegen {

using ObjectID = unsigned long long;

struct OSMWayNode {
  ObjectID id;
  Coordinate coordinate;
};

enum class OnewayDirection { None, Forward, Backward };

struct OSMWay {
  std::vector<OSMWayNode> nodes;
  OnewayDirection oneway_direction = OnewayDirection::None;
};

struct OSMDataCollector : public osmium::handler::Handler {
public:
  void way(const osmium::Way &way);

  void CollectFrom(const std::string &filename);
private:
  OnewayDirection getOnewayDirection(const osmium::Way &way) const;
  bool isWayAccessibleByAuto(const osmium::Way &way) const;

public:
  std::unordered_map<ObjectID, size_t> intersections;
  std::vector<OSMWay> ways;
};

} // namespace tilegen
} // namespace mama
