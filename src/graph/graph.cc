#include "mama.hpp"

#include <s2/encoded_s2shape_index.h>
#include <s2/s2closest_edge_query.h>
#include "graph/tile_level.hpp"
#include "state.pb.h"
#include "tile.pb.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <s2/s2cap.h>
#include <s2/s2earth.h>
#include <s2/s2latlng.h>
#include <s2/s2region_coverer.h>
#include <s2/s2shapeutil_coding.h>
#include <s2/util/coding/coder.h>

namespace mama {

bool operator<(const EdgeId &lhs, const EdgeId &rhs) {
  return std::tie(lhs.tile_id, lhs.edge_index) <
         std::tie(rhs.tile_id, rhs.edge_index);
}

class Tile {
public:
  explicit Tile(TileId tile_id, const std::string &path) : tile_id_(tile_id) {
    std::ifstream str(path);
    header_.ParseFromIstream(&str);

    decoder_ = std::make_unique<Decoder>(header_.shape_spatial_index().data(),
                                         header_.shape_spatial_index().size());
    spatial_index_.Init(decoder_.get(),
                        s2shapeutil::LazyDecodeShapeFactory(decoder_.get()));
  }

  std::vector<Projection> Project(const Coordinate &coordinate,
                                  double radius_m) {
    S2ClosestEdgeQuery query(&spatial_index_);
    query.mutable_options()->set_max_distance(
        S2Earth::MetersToChordAngle(radius_m));
    auto point = coordinate.AsS2LatLng().Normalized().ToPoint();
    S2ClosestEdgeQuery::PointTarget target(point);

    std::vector<Projection> results;
    for (const auto &result : query.FindClosestEdges(&target)) {
      auto coordinate = S2LatLng(query.Project(point, result));
      Projection projection;
      projection.point_on_graph.edge_id.tile_id = tile_id_;
      projection.point_on_graph.edge_id.edge_index = result.shape_id();

      auto shape = spatial_index_.shape(result.shape_id());
      assert(shape);

      // TODO: how do we optimize this?
      {
        double meters_offset = 0.0;
        for (size_t edge_id = 0; edge_id < result.edge_id(); ++edge_id) {
          auto edge = shape->edge(edge_id);
          auto a = S2LatLng(edge.v0).Normalized();
          auto b = S2LatLng(edge.v1).Normalized();
          auto edge_length = S2Earth::ToMeters(a.GetDistance(b));
          meters_offset += edge_length;
        }

        meters_offset +=
            S2Earth::ToMeters(S2LatLng(shape->edge(result.edge_id()).v0)
                                  .Normalized()
                                  .GetDistance(coordinate.Normalized()));

        projection.point_on_graph.offset =
            meters_offset / edges(result.shape_id()).length();
        projection.point_on_graph.offset =
            std::clamp(projection.point_on_graph.offset, 0.0, 1.0);
      }

      {
        auto edge = shape->edge(result.edge_id());
        auto a = Coordinate::FromS2LatLng(S2LatLng(edge.v0));
        auto b = Coordinate::FromS2LatLng(S2LatLng(edge.v1));
        projection.bearing_deg = a.BearingTo(b);
      }

      projection.coordinate = Coordinate::FromS2LatLng(coordinate);
      projection.distance_m = S2Earth::ToMeters(result.distance());
      results.push_back(projection);
    }

    return results;
  }

  const auto &edges() const { return header_.edges(); }
  const tile::Edge &edges(int index) const { return header_.edges(index); }

  const auto &nodes() const { return header_.nodes(); }
  const auto &nodes(int index) const { return header_.nodes(index); }

  const auto &header() const { return header_; }

private:
  TileId tile_id_;
  tile::Header header_;
  std::unique_ptr<Decoder> decoder_;
  EncodedS2ShapeIndex spatial_index_;
};

Graph::Graph(const std::string &tiles_folder) : tiles_folder_(tiles_folder) {}

std::vector<double> Graph::PathDistance(const PointOnGraph &from,
                                        const std::vector<PointOnGraph> &to,
                                        const PathOptions &options) {
  struct Distance {
    double distance = 0.0;
    EdgeId edge_id;

    bool operator<(const Distance &rhs) const {
      return distance > rhs.distance;
    }
  };

  std::vector<double> results;
  results.resize(to.size(), std::numeric_limits<double>::max());

  std::map<EdgeId, size_t> to_find;
  for (size_t index = 0; index < to.size(); ++index) {
    to_find[to[index].edge_id] = index;
  }

  std::set<EdgeId> visited;
  std::priority_queue<Distance> queue;
  auto init_edge = GetEdge(from.edge_id);
  if (!init_edge) {
    return results;
  }

  queue.push({init_edge->length() * (1.0 - from.offset), from.edge_id});
  while (!queue.empty()) {
    auto current = queue.top();
    queue.pop();

    if (to_find.find(current.edge_id) != to_find.end()) {
      auto index = to_find[current.edge_id];

      auto finish_edge = GetEdge(to[index].edge_id);
      auto distance =
          current.distance - finish_edge->length() * (1.0 - to[index].offset);

      // this is needed to handle case when `from` and `to` are on the same
      // edge, but `to` is before `from` by offset
      if (distance >= 0.0) {
        to_find.erase(current.edge_id);
        results[index] = distance;
        continue;
      }
    }

    const auto edge = GetEdge(current.edge_id);
    auto target_node =
        GetTargetNode(current.edge_id.tile_id, edge->target_node_id());
    if (!target_node) {
      continue;
    }

    auto adjacent_edge_ids = GetAdjacentEdges(*target_node);
    for (const auto &adjacent_edge_id : adjacent_edge_ids) {
      if (visited.find(adjacent_edge_id) != visited.end()) {
        continue;
      }
      visited.insert(adjacent_edge_id);

      auto edge = GetEdge(adjacent_edge_id);
      if (!edge) {
        continue;
      }

      auto distance = current.distance + edge->length();
      if (distance > options.max_distance_m) {
        continue;
      }
      queue.push({distance, adjacent_edge_id});
    }
  }

  return results;
}

std::vector<Projection> Graph::Project(const Coordinate &coordinate,
                                       double radius_m) {
  // TODO: make coverer class member
  S2RegionCoverer::Options options;
  options.set_fixed_level(graph::kTileLevel);
  S2RegionCoverer coverer(options);

  S2Cap cap(coordinate.AsS2LatLng().Normalized().ToPoint(),
            S2Earth::MetersToChordAngle(radius_m));
  std::vector<S2CellId> cells;
  coverer.GetCovering(cap, &cells);

  std::vector<Projection> results;
  for (const auto cellId : cells) {
    assert(cellId.level() == graph::kTileLevel);

    auto tile = GetTile(cellId.id());
    if (!tile) {
      continue;
    }
    auto tile_results = tile->Project(coordinate, radius_m);
    results.insert(results.end(), tile_results.begin(), tile_results.end());
  }
  return results;
}

const tile::Edge *Graph::GetEdge(const EdgeId &edge_id) {
  auto tile = GetTile(edge_id.tile_id);
  return tile ? &tile->edges(static_cast<int>(edge_id.edge_index)) : nullptr;
}

const tile::Node *Graph::GetNode(const NodeId &node_id) {
  auto tile = GetTile(node_id.tile_id);
  return tile ? &tile->nodes(static_cast<int>(node_id.node_index)) : nullptr;
}

std::optional<NodeId> Graph::GetTargetNode(TileId tile_id, ssize_t node_index) {
  if (node_index >= 0) {
    return {{tile_id, static_cast<uint32_t>(node_index)}};
  } else {
    // TODO: -1 while encoding?
    auto tile = GetTile(tile_id);
    if (!tile) {
      return {};
    }
    const auto &neighbor_tile_node =
        tile->header().neighbour_tile_nodes(static_cast<int>(-node_index - 1));
    return {{neighbor_tile_node.tile_id(), neighbor_tile_node.node_id()}};
  }
}

std::vector<EdgeId> Graph::GetAdjacentEdges(const NodeId &node_id) {
  auto node = GetNode(node_id);
  if (!node) {
    return {};
  }
  std::vector<EdgeId> results;
  for (const auto edge_index : node->adjacent_edges()) {
    results.push_back({node_id.tile_id, edge_index});
  }
  return results;
}

std::shared_ptr<Tile> Graph::GetTile(TileId tile_id) {
  auto itr = tiles_.find(tile_id);
  if (itr != tiles_.end()) {
    return itr->second;
  }
  auto tile_path = tiles_folder_ + "/" + std::to_string(tile_id) + ".tile";
  if (!std::filesystem::exists(tile_path)) {
    tiles_.insert({tile_id, {}});
    return {};
  }
  auto tile = std::make_shared<Tile>(tile_id, tile_path);
  tiles_.insert({tile_id, tile});
  return tile;
}

} // namespace mama
