#include "osm_data_collector.hpp"
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/util/progress_bar.hpp>
#include <osmium/visitor.hpp>

namespace mama {
namespace tilegen {

void OSMDataCollector::CollectFrom(const std::string& filename) {
  osmium::io::Reader reader{filename, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};
  osmium::ProgressBar progress{reader.file_size(), osmium::isatty(2)};

  using Index = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
  using LocationHandler = osmium::handler::NodeLocationsForWays<Index>;

  Index index;
  LocationHandler location_handler{index};

  while (osmium::memory::Buffer buffer = reader.read()) {
    osmium::apply(buffer, location_handler, *this);
    progress.update(reader.offset());
  }

  progress.done();
}

void OSMDataCollector::way(const osmium::Way& way) {
  if (!isWayAccessibleByAuto(way)) {
    return;
  }

  OSMWay internal_way;
  for (const auto& node : way.nodes()) {
    ++intersections[node.ref()];
    internal_way.nodes.push_back({static_cast<ObjectID>(node.ref()), {node.location().lon(), node.location().lat()}});
  }
  internal_way.oneway_direction = getOnewayDirection(way);

  ways.emplace_back(std::move(internal_way));
}

OnewayDirection OSMDataCollector::getOnewayDirection(const osmium::Way& way) const {
  const char* oneway = way.tags()["oneway"];
  if (oneway == nullptr) {
    return OnewayDirection::None;
  }
  if (strcmp(oneway, "yes") == 0) {
    return OnewayDirection::Forward;
  }
  if (strcmp(oneway, "-1") == 0) {
    return OnewayDirection::Backward;
  }
  return OnewayDirection::None;
}

bool OSMDataCollector::isWayAccessibleByAuto(const osmium::Way& way) const {
  static std::unordered_set<std::string> kAccessibleTags = {
      "motorway",     "trunk",          "primary",       "secondary",     "tertiary",
      "unclassified", "residential",    "service",       "motorway_link", "trunk_link",
      "primary_link", "secondary_link", "tertiary_link", "living_street", "road"};
  const char* highway = way.tags()["highway"];
  if (highway == nullptr) {
    return false;
  }
  return kAccessibleTags.contains(highway);
}

}  // namespace tilegen
}  // namespace mama
