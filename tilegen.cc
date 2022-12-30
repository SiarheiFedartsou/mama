#include <cinttypes>
#include <cstdint>
#include <cstdio>

#include <s2/base/commandlineflags.h>
#include "s2/s2earth.h"
#include "absl/flags/flag.h"
#include "s2/s1chord_angle.h"
#include "s2/s2closest_point_query.h"
#include "s2/s2point_index.h"
#include "s2/s2polyline.h"
#include "s2/util/coding/coder.h"
#include "s2/s2shapeutil_coding.h"
#include "s2/mutable_s2shape_index.h"
#include "tile.pb.h"
#include <osmium/geom/haversine.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>
#include <osmium/util/progress_bar.hpp>
#include <osmium/geom/geojson.hpp>
#include <stdexcept>
#include <fstream>

constexpr int kTileLevel = 11;
using ObjectID = unsigned long long;

struct Coordinate {
  double x{};
  double y{};

  double lng() const {
    return x;
  }

  double lat() const {
    return y;
  }

  osmium::geom::Coordinates asOSMCoord() const {
    return osmium::geom::Coordinates(x, y);
  }

  S2LatLng asS2LatLng() const {
    return S2LatLng::FromDegrees(y, x);
  }
};


struct WayNode {
  ObjectID id;
  Coordinate coordinate;
};

enum class OnewayDirection { None, Forward, Backward };

struct Way {
  std::vector<WayNode> nodes;
  OnewayDirection oneway_direction = OnewayDirection::None;
};

struct DataCollector : public osmium::handler::Handler {
public:
  void way(const osmium::Way &way) {\
    if (!isWayAccessibleByAuto(way)) {
      return;
    }

    Way internal_way;
    for (const auto &node : way.nodes()) {
      ++intersections[node.ref()];
      internal_way.nodes.push_back(
          {static_cast<ObjectID>(node.ref()),
           {node.location().lon(), node.location().lat()}});
    }
    ways.emplace_back(std::move(internal_way));
  }

private:
  bool isWayAccessibleByAuto(const osmium::Way &way) const {
    static std::unordered_set<std::string> kAccessibleTags = {
        "motorway", "trunk", "primary", "secondary", "tertiary", "unclassified",
        "residential", "service", "motorway_link", "trunk_link", "primary_link",
        "secondary_link", "tertiary_link", "living_street", "road"};
    const char *highway = way.tags()["highway"];
    if (highway == nullptr) {
      return false;
    }
    return kAccessibleTags.contains(highway);
  }
public:
  std::unordered_map<ObjectID, size_t> intersections;
  std::vector<Way> ways;
};


struct Edge {
  ObjectID from;
  ObjectID to;
  double distance;
  std::vector<Coordinate> shape;
};

using TileId = uint64_t;

struct Node {
  Coordinate coordinate;
  ObjectID id;
  std::vector<size_t> adjacent_edges;

  TileId getTileId() const {
    S2CellId cellId{S2LatLng::FromDegrees(coordinate.lat(), coordinate.lng())};
    S2CellId tileCellId = cellId.parent(kTileLevel);
    return tileCellId.id();
  }
};


class GraphBuilder {
public:
  void build(const DataCollector &data_collector) {
    for (const auto &way : data_collector.ways) {
      std::vector<Coordinate> shape;
      double distance = 0.0;
      ObjectID start_node_id = -1;

      for (size_t i = 0; i < way.nodes.size(); ++i) {
        if (start_node_id == -1) {
          start_node_id = way.nodes[i].id;
          shape = {way.nodes[i].coordinate};
        } else if (data_collector.intersections.at(way.nodes[i].id) > 1) {
          shape.push_back(way.nodes[i].coordinate);

          Edge edge;
          edge.from = start_node_id;
          edge.to = way.nodes[i].id;
          edge.shape = std::move(shape);
          addEdge(std::move(edge), way);

          shape = {way.nodes[i].coordinate};
          distance = 0.0;


          start_node_id = way.nodes[i].id;
        } else {
          shape.push_back(way.nodes[i].coordinate);
        }
        
        if (shape.size() > 1) {
          const auto &node1 = shape[shape.size() - 2];
          const auto &node2 = shape[shape.size() - 1];
          distance += osmium::geom::haversine::distance(
              node1.asOSMCoord(), node2.asOSMCoord());
        }
      }
      if (distance > 0) {
        Edge edge;
        edge.from = start_node_id;
        edge.to = way.nodes.back().id;
        edge.shape = std::move(shape);
        addEdge(std::move(edge), way);
      }
    }
    std::cout << "Number of edges: " << edges.size() << std::endl;
  }
  std::vector<Edge> edges;
  std::unordered_map<ObjectID, Node> nodes;

private:

  void addNode(ObjectID nodeId, Coordinate coordinate) {
      Node& node = nodes[nodeId];
      node.id = nodeId;
      node.coordinate = coordinate;
      node.adjacent_edges.push_back(edges.size() - 1);
  }

  void addEdge(Edge &&edge, const Way& fromWay) {
    auto fromCoordinate = edge.shape.front();
    auto toCoordinate = edge.shape.back();
    auto fromId = edge.from;
    auto toId = edge.to;
    if (fromWay.oneway_direction == OnewayDirection::Forward) {
      edges.push_back(std::move(edge));

      addNode(fromId, fromCoordinate);
    } else if (fromWay.oneway_direction == OnewayDirection::Backward) {
      std::reverse(edge.shape.begin(), edge.shape.end());
      std::swap(edge.from, edge.to);
      edges.push_back(std::move(edge));
      addNode(toId, toCoordinate);
    } else {
      edges.push_back(edge);

      addNode(fromId, fromCoordinate);
      std::reverse(edge.shape.begin(), edge.shape.end());
      std::swap(edge.from, edge.to);
      edges.push_back(std::move(edge));
      addNode(toId, toCoordinate);
    }
  }
};


struct TileBuilder {
  size_t edge_counter = 0;
  size_t node_counter = 0;

  void addNode(const Node &node, const std::vector<Edge>& edges, const std::unordered_map<ObjectID, Node>& nodes) {
    auto* pbfNode = header.mutable_nodes(getLocalNodeIndex(node));

    for (auto adjacent_edge_index: node.adjacent_edges) {
      auto local_index = getLocalEdgeIndex(adjacent_edge_index, edges[adjacent_edge_index], nodes);
      pbfNode->add_adjacent_edges(local_index);
    }
  }

  size_t getLocalEdgeIndex(size_t global_index, const Edge& edge, const std::unordered_map<ObjectID, Node>& nodes) {
    auto local_index_itr = global_to_tile_edge_index.find(global_index);
    if (local_index_itr == global_to_tile_edge_index.end()) {
      local_index_itr = global_to_tile_edge_index.emplace(global_index, addEdge(edge, nodes)).first;
    }
    return local_index_itr->second;
  }

  ssize_t getLocalNodeIndex(const Node& node) {
    if (node.getTileId() != tile_id) {
      // add +1 to avoid ambiguity in the case of 0 index
      return -(addNeighbourTileNode(node) + 1);
    }
    auto local_index_itr = node_id_to_tile_node_index.find(node.id);
    if (local_index_itr == node_id_to_tile_node_index.end()) {
      local_index_itr = node_id_to_tile_node_index.emplace(node.id, addNode()).first;
    }
    return local_index_itr->second;
  }

  ssize_t addNeighbourTileNode(const Node& node) {
    auto pbfNode = header.add_neighbour_tile_nodes();
    neighbour_nodes.emplace_back(NeighbourNode{node.getTileId(), node.id});
    return header.neighbour_tile_nodes_size() - 1;
  }

  size_t addEdge(const Edge& edge, const std::unordered_map<ObjectID, Node>& nodes) {
    auto pbfEdge = header.add_edges();
    pbfEdge->set_length(edge.distance);
    pbfEdge->set_target_node_id(getLocalNodeIndex(nodes.at(edge.to)));

    std::vector<S2LatLng> latlngs;
    for (const auto &node : edge.shape) {
      latlngs.emplace_back(node.asS2LatLng());
    }

    auto polyline = std::make_unique<S2Polyline>(absl::Span<const S2LatLng>{latlngs});
    polylines.emplace_back(std::move(polyline));
    auto shape_id = edge_index.Add(std::make_unique<S2Polyline::Shape>(polylines.back().get()));

    pbfEdge->set_shape_id(shape_id);

    return header.edges_size() - 1;
  }

  size_t addNode() {
    auto pbfNode = header.add_nodes();
    return header.nodes_size() - 1;
  }




  void finish(const std::unordered_map<TileId, TileBuilder>& tile_builders, const std::string& output_folder) {
      fixNeighbourTileNodes(tile_builders);

      Encoder encoder;
      s2shapeutil::CompactEncodeTaggedShapes(edge_index, &encoder);
      edge_index.Encode(&encoder);
      *header.mutable_shape_spatial_index() = {encoder.base(), encoder.length()};

      std::cerr << tile_id << " " << header.ByteSizeLong() << std::endl;

      std::ofstream out(output_folder + "/" + std::to_string(tile_id) + ".tile");
      header.SerializeToOstream(&out);
  }

  std::unordered_map<size_t, size_t> global_to_tile_edge_index;
  std::unordered_map<ObjectID, size_t> node_id_to_tile_node_index;
  
  MutableS2ShapeIndex edge_index;
  TileId tile_id;
  std::vector<std::unique_ptr<S2Polyline>> polylines;

  struct NeighbourNode {
    TileId tile_id;
    ObjectID node_id;
  };

  std::vector<NeighbourNode> neighbour_nodes;


  tile::Header header;

private:
  void fixNeighbourTileNodes(const std::unordered_map<TileId, TileBuilder>& otherBuilders) {
    assert(neighbour_nodes.size() == header.neighbour_tile_nodes_size());
    for (size_t index = 0; index < neighbour_nodes.size(); ++index) {
      auto& pbfNode = *header.mutable_neighbour_tile_nodes(index);
      auto& neighbour_node = neighbour_nodes[index];
      auto other_tile_id = neighbour_node.tile_id;
      auto other_node_id = neighbour_node.node_id;

      auto other_tile_builder_itr = otherBuilders.find(other_tile_id);
      if (other_tile_builder_itr == otherBuilders.end()) {
        throw std::runtime_error("Could not find tile " + std::to_string(other_tile_id));
      }
      const auto& other_tile_builder = other_tile_builder_itr->second;
      auto other_node_index = other_tile_builder.node_id_to_tile_node_index.find(other_node_id);
      if (other_node_index == other_tile_builder.node_id_to_tile_node_index.end()) {
        throw std::runtime_error("Could not find node " + std::to_string(other_node_id) + " in tile " + std::to_string(other_tile_id));
        continue;
      }
      pbfNode.set_tile_id(other_tile_id);
      pbfNode.set_node_id(other_node_index->second);
    }
  }
};

int main(int argc, char **argv) {
  std::cerr << S2CellId::lsb_for_level(kTileLevel) << std::endl;;
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " OSMFILE OUTPUTFOLDER\n";
    return 1;
  }

  std::string output_folder = argv[2];

  try {
    osmium::io::Reader reader{argv[1], osmium::osm_entity_bits::node |
                                           osmium::osm_entity_bits::way};
    osmium::ProgressBar progress{reader.file_size(), osmium::isatty(2)};

    using Index = osmium::index::map::FlexMem<osmium::unsigned_object_id_type,
                                              osmium::Location>;
    using LocationHandler = osmium::handler::NodeLocationsForWays<Index>;

    Index index;
    LocationHandler location_handler{index};

    DataCollector data_collector;

    while (osmium::memory::Buffer buffer = reader.read()) {
        osmium::apply(buffer, location_handler, data_collector);
        progress.update(reader.offset());
    }


    progress.done();

    GraphBuilder builder;
    builder.build(data_collector);




    std::unordered_map<TileId, TileBuilder> tile_builders;
    for (const auto& [nodeId, node] : builder.nodes) {
      TileBuilder& tile_builder = tile_builders[node.getTileId()];
      tile_builder.tile_id = node.getTileId();
      tile_builder.addNode(node, builder.edges, builder.nodes);
    }

    for (auto& [tile_id, tile_builder] : tile_builders) {
      tile_builder.finish(tile_builders, output_folder);
    }

  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}