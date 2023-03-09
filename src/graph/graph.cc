#include "mama.hpp"

#include "graph/tile_level.hpp"
#include "state.pb.h"
#include "tile.pb.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>

#include <s2/s2latlng.h>
#include <s2/s2region_coverer.h>
#include <s2/s2shapeutil_coding.h>
#include <s2/util/coding/coder.h>
#include <s2/encoded_s2shape_index.h>
#include <s2/s2closest_edge_query.h>


namespace mama {

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

  bool shortest_path(const tile::Edge& from_edge, size_t to_edge_index, double* distance) {
    assert(distance);
    if (!from_edge.has_distance_table()) {
      return false;
    }

    const auto& distance_table = from_edge.distance_table();
    auto itr = std::lower_bound(distance_table.edge_id().begin(), distance_table.edge_id().end(), to_edge_index);
    if (itr == distance_table.edge_id().end() || *itr != to_edge_index) {
      return std::numeric_limits<double>::max();
    }
    *distance = static_cast<double>(distance_table.distance(std::distance(distance_table.edge_id().begin(), itr)));
    return true;
  }
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

  auto from_tile = GetTile(from.edge_id.tile_id);


  std::vector<double> results;
  results.resize(to.size(), std::numeric_limits<double>::max());


  absl::flat_hash_map<EdgeId, size_t> to_find;
  to_find.reserve(to.size());
  for (size_t index = 0; index < to.size(); ++index) {
     if (to[index].edge_id.tile_id == from.edge_id.tile_id) {
      if (to[index].edge_id.edge_index == from.edge_id.edge_index && to[index].offset >= from.offset) {
        auto edge = from_tile->edges(to[index].edge_id.edge_index);
        results[index] = edge.length() * (to[index].offset - from.offset);
      } else {
        bool has_distance_table = from_tile->shortest_path(from_tile->edges(from.edge_id.edge_index), to[index].edge_id.edge_index, &results[index]);
        if (has_distance_table) {
          const auto& begin_edge = from_tile->edges(from.edge_id.edge_index);
          const auto& end_edge = from_tile->edges(to[index].edge_id.edge_index);
          if (results[index] != std::numeric_limits<double>::max()) {
            results[index] += begin_edge.length() * (1.0 - from.offset);
            results[index] += end_edge.length() * to[index].offset;
          }

          // TODO: do we really care about this?
          if (results[index] > options.max_distance_m) {
            results[index] = std::numeric_limits<double>::max();
          }

        } else {
          to_find[to[index].edge_id] = index;
        }

       }
      // to_find[to[index].edge_id] = index;
    } else {
      to_find[to[index].edge_id] = index;
    }
  }

  if (to_find.empty()) {
    return results;
  }


  auto init_edge = GetEdge(from.edge_id);
  if (!init_edge) {
    return results;
  }

  absl::flat_hash_set<EdgeId> visited;
  std::priority_queue<Distance> queue;

  queue.push({init_edge->length() * (1.0 - from.offset), from.edge_id});
  while (!queue.empty()) {
    auto current = queue.top();
    queue.pop();

    const auto edge = GetEdge(current.edge_id);
    
    auto to_find_itr = to_find.find(current.edge_id);
    if (to_find_itr != to_find.end()) {
      auto index = to_find_itr->second;

      auto distance =
          current.distance - edge->length() * (1.0 - to[index].offset);

      // TODO: is it really needed ??? It seems that it is not, driver could get there just due to a U-turn
      // this is needed to handle case when `from` and `to` are on the same
      // edge, but `to` is before `from` by offset
      if (distance >= 0.0) {
        to_find.erase(to_find_itr);
        results[index] = distance;
        if (to_find.empty()) {
          break;
        }
      }
    }

    auto target_node =
        GetTargetNode(current.edge_id.tile_id, edge->target_node_id());
    if (!target_node) {
      continue;
    }

    auto node = GetNode(*target_node);

    // it shouldn't be null, because we already checked it above
    assert(node); 

    for (const auto edge_index: node->adjacent_edges()) {
      EdgeId adjacent_edge_id{target_node->tile_id, edge_index};
      
      if (!visited.insert(adjacent_edge_id).second) {
        continue;
      }

      auto edge = GetEdge(adjacent_edge_id);
      assert(edge);

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
  auto cells = coverer_.GetCovering(coordinate, radius_m);

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
    auto tile = GetTile(tile_id);
    if (!tile) {
      return {};
    }
    // indexes in `neighbour_tile_nodes` are encoded as negative values,
    // 0 is used for usual nodes, so we start counting from -1 (that's why have this -1 here)
    const auto &neighbor_tile_node =
        tile->header().neighbour_tile_nodes(static_cast<int>(-node_index - 1));
    return {{neighbor_tile_node.tile_id(), neighbor_tile_node.node_id()}};
  }
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
