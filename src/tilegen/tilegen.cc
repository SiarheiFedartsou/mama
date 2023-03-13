#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "base/coordinate.hpp"
#include "base/log.hpp"
#include "graph/tile_level.hpp"
#include "osm_data_collector.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include "options.hpp"
#include "s2/mutable_s2shape_index.h"
#include "s2/s1chord_angle.h"
#include "s2/s2closest_point_query.h"
#include "s2/s2earth.h"
#include "s2/s2point_index.h"
#include "s2/s2polyline.h"
#include "s2/s2shapeutil_coding.h"
#include "s2/util/coding/coder.h"
#include "tile.pb.h"

namespace mama {
namespace tilegen {

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
    S2CellId tileCellId = cellId.parent(graph::kTileLevel);
    return tileCellId.id();
  }
};

class GraphBuilder {
 public:
  void build(const OSMDataCollector& data_collector) {
    for (const auto& way : data_collector.ways) {
      double distance = 0.0;

      assert(!way.nodes.empty());
      ObjectID start_node_id = way.nodes.front().id;
      std::vector<Coordinate> shape = {way.nodes.front().coordinate};
      for (size_t i = 1; i < way.nodes.size(); ++i) {
        shape.push_back(way.nodes[i].coordinate);
        distance += shape.rbegin()->Distance(*(shape.rbegin() + 1));
        if (data_collector.intersections.at(way.nodes[i].id) > 1) {
          Edge edge;
          edge.from = start_node_id;
          edge.to = way.nodes[i].id;
          edge.shape = std::move(shape);
          edge.distance = distance;
          addEdge(std::move(edge), way);

          shape = {way.nodes[i].coordinate};
          distance = 0.0;

          start_node_id = way.nodes[i].id;
        }
      }
      if (distance > 0) {
        Edge edge;
        edge.from = start_node_id;
        edge.to = way.nodes.back().id;
        edge.shape = std::move(shape);
        edge.distance = distance;
        addEdge(std::move(edge), way);
      }
    }
  }
  std::vector<Edge> edges;
  std::unordered_map<ObjectID, Node> nodes;

 private:
  Node& addNode(ObjectID nodeId, Coordinate coordinate) {
    Node& node = nodes[nodeId];
    node.id = nodeId;
    node.coordinate = coordinate;
    return node;
  }

  void addEdge(Edge&& edge, const OSMWay& fromWay) {
    {
      auto length = 0.0;
      for (size_t i = 1; i < edge.shape.size(); ++i) {
        length += edge.shape[i - 1].Distance(edge.shape[i]);
      }
      assert(std::abs(length - edge.distance) < 1e-2);
    }

    auto fromCoordinate = edge.shape.front();
    auto toCoordinate = edge.shape.back();
    auto fromId = edge.from;
    auto toId = edge.to;
    if (fromWay.oneway_direction == OnewayDirection::Forward) {
      edges.push_back(std::move(edge));

      addNode(fromId, fromCoordinate).adjacent_edges.push_back(edges.size() - 1);
      addNode(toId, toCoordinate);

    } else if (fromWay.oneway_direction == OnewayDirection::Backward) {
      std::reverse(edge.shape.begin(), edge.shape.end());
      std::swap(edge.from, edge.to);
      edges.push_back(std::move(edge));

      addNode(fromId, fromCoordinate);
      addNode(toId, toCoordinate).adjacent_edges.push_back(edges.size() - 1);
    } else {
      edges.push_back(edge);

      addNode(fromId, fromCoordinate).adjacent_edges.push_back(edges.size() - 1);

      std::reverse(edge.shape.begin(), edge.shape.end());
      std::swap(edge.from, edge.to);
      edges.push_back(std::move(edge));

      addNode(toId, toCoordinate).adjacent_edges.push_back(edges.size() - 1);
    }
  }
};

class TileBuilder {
 public:
  explicit TileBuilder(TileId tile_id) : tile_id(tile_id) {}

  void addNode(const Node& node, const std::vector<Edge>& edges, const std::unordered_map<ObjectID, Node>& nodes) {
    auto* pbfNode = header.mutable_nodes(getLocalNodeIndex(node));

    for (auto adjacent_edge_index : node.adjacent_edges) {
      auto local_index = getLocalEdgeIndex(adjacent_edge_index, edges[adjacent_edge_index], nodes);
      pbfNode->add_adjacent_edges(local_index);
    }
  }

  void finish(const std::unordered_map<TileId, TileBuilder>& tile_builders, const std::string& output_folder) {
    fixNeighbourTileNodes(tile_builders);

    Encoder encoder;
    s2shapeutil::CompactEncodeTaggedShapes(edge_index, &encoder);
    edge_index.Encode(&encoder);
    *header.mutable_shape_spatial_index() = {encoder.base(), encoder.length()};

    MAMA_INFO("Generated tile {} with {} edges and {} nodes. Size is {} bytes.", tile_id, header.edges_size(),
              header.nodes_size(), header.ByteSizeLong());

    std::ofstream out(output_folder + "/" + std::to_string(tile_id) + ".tile");
    header.SerializeToOstream(&out);
  }

 private:
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
    pbfEdge->set_length(static_cast<uint32_t>(std::round(edge.distance)));
    pbfEdge->set_target_node_id(getLocalNodeIndex(nodes.at(edge.to)));
    std::vector<S2LatLng> latlngs;
    for (const auto& node : edge.shape) {
      latlngs.emplace_back(node.AsS2LatLng());
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
        throw std::runtime_error("Could not find node " + std::to_string(other_node_id) + " in tile " +
                                 std::to_string(other_tile_id));
        continue;
      }
      pbfNode.set_tile_id(other_tile_id);
      pbfNode.set_node_id(other_node_index->second);
    }
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

  mama::tile::Header header;
};

void BuildDistanceTables(const std::vector<TileId> tile_ids, const Options& cli_options) {
  struct EdgeInfo {
    size_t edge_index = 0;
    uint32_t distance = 0;

    bool operator<(const EdgeInfo& rhs) const { return distance > rhs.distance; }
  };

  struct DistanceTableEntry {
    uint32_t edge_index = 0;
    uint32_t distance = 0;
  };

  for (auto tile_id : tile_ids) {
    std::ifstream tile_ifs(cli_options.output_folder + "/" + std::to_string(tile_id) + ".tile");
    mama::tile::Header header;
    header.ParseFromIstream(&tile_ifs);

    for (size_t edge_index = 0; edge_index < header.edges_size(); ++edge_index) {
      std::vector<DistanceTableEntry> distance_table;

      std::priority_queue<EdgeInfo> queue;
      std::unordered_set<size_t> visited_edges;

      queue.push(EdgeInfo{edge_index, 0});

      while (!queue.empty()) {
        auto current = queue.top();
        queue.pop();

        auto target_node_id = header.edges(current.edge_index).target_node_id();
        // TODO: what if shortest path goes through another tile?
        // TODO: it means it is in another tile, but we need a way to also precompute distances to edges in other tiles
        if (target_node_id < 0) {
          continue;
        }
        auto target_node = header.nodes(target_node_id);
        for (auto adjacent_edge_index : target_node.adjacent_edges()) {
          if (!visited_edges.insert(adjacent_edge_index).second) {
            continue;
          }

          uint32_t length = static_cast<uint32_t>(header.edges(adjacent_edge_index).length());

          distance_table.push_back(DistanceTableEntry{adjacent_edge_index, current.distance});
          auto new_distance = current.distance + length;
          if (new_distance > *cli_options.max_precompute_path_length) {
            continue;
          }
          queue.push(EdgeInfo{adjacent_edge_index, new_distance});
        }
      }

      std::sort(distance_table.begin(), distance_table.end(),
                [](const DistanceTableEntry& a, const DistanceTableEntry& b) { return a.edge_index < b.edge_index; });

      auto& pbf_distance_table = *header.mutable_edges(edge_index)->mutable_distance_table();
      for (auto& entry : distance_table) {
        pbf_distance_table.add_edge_id(entry.edge_index);
        pbf_distance_table.add_distance(entry.distance);
      }
    }

    std::ofstream out(cli_options.output_folder + "/" + std::to_string(tile_id) + ".tile");
    header.SerializeToOstream(&out);
  }
}

}  // namespace tilegen
}  // namespace mama

int main(int argc, char** argv) {
  using namespace mama::tilegen;

  Options cli_options = Options::Parse(argc, argv);

  mama::base::InitializeLogging();

  try {
    OSMDataCollector data_collector;
    data_collector.CollectFrom(cli_options.osm_file);

    GraphBuilder builder;
    builder.build(data_collector);

    std::unordered_map<TileId, TileBuilder> tile_builders;
    for (const auto& [nodeId, node] : builder.nodes) {
      auto tile_builder_itr = tile_builders.find(node.getTileId());
      if (tile_builder_itr == tile_builders.end()) {
        tile_builder_itr = tile_builders.emplace(node.getTileId(), node.getTileId()).first;
      }
      tile_builder_itr->second.addNode(node, builder.edges, builder.nodes);
    }

    for (auto& [tile_id, tile_builder] : tile_builders) {
      tile_builder.finish(tile_builders, cli_options.output_folder);
    }

    std::vector<TileId> tile_ids;
    tile_ids.reserve(tile_builders.size());
    for (const auto& [tile_id, _] : tile_builders) {
      tile_ids.push_back(tile_id);
    }

    if (cli_options.max_precompute_path_length) {
      BuildDistanceTables(tile_ids, cli_options);
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}