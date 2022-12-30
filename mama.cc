#include "s2/encoded_s2shape_index.h"
#include "s2/s2closest_edge_query.h"

#include "s2/s2cap.h"
#include "s2/s2earth.h"
#include "s2/s2latlng.h"
#include "s2/s2region_coverer.h"
#include "s2/s2shapeutil_coding.h"
#include "s2/util/coding/coder.h"
#include "tile.pb.h"
#include "state.pb.h"

#include <filesystem>
#include <fstream>
#include <iostream>

struct Coordinate {
  double lon = 0.0;
  double lat = 0.0;
};

using TileId = uint64_t;

struct EdgeId {
  TileId tile_id;
  uint32_t edge_index = 0;
};

bool operator<(const EdgeId &lhs, const EdgeId &rhs) {
  return std::tie(lhs.tile_id, lhs.edge_index) <
         std::tie(rhs.tile_id, rhs.edge_index);
}

struct NodeId {
  TileId tile_id;
  uint32_t node_index = 0;
};

struct PointOnGraph {
  EdgeId edge_id;
  // range 0-1
  double offset = 0.0;
};

struct Projection {
  PointOnGraph point_on_graph;
  double distance_m = 0.0;
  Coordinate coordinate;
  double bearing_deg = 0.0;
};

class Tile {
public:
  explicit Tile(TileId tile_id, const std::string &path) : tile_id_(tile_id) {
    std::ifstream str(path);
    header_.ParseFromIstream(&str);
    std::cerr << header_.edges_size() << std::endl;

    decoder_ = std::make_unique<Decoder>(header_.shape_spatial_index().data(),
                                         header_.shape_spatial_index().size());
    spatial_index_.Init(decoder_.get(),
                        s2shapeutil::LazyDecodeShapeFactory(decoder_.get()));

    std::cerr << spatial_index_.num_shape_ids() << std::endl;
  }

  std::vector<Projection> Project(const Coordinate &coordinate,
                                  double radius_m) {
    S2ClosestEdgeQuery query(&spatial_index_);
    query.mutable_options()->set_max_distance(
        S2Earth::MetersToChordAngle(radius_m));
    auto point = S2LatLng::FromDegrees(coordinate.lat, coordinate.lon)
                     .Normalized()
                     .ToPoint();
    S2ClosestEdgeQuery::PointTarget target(point);

    std::vector<Projection> results;
    for (const auto &result : query.FindClosestEdges(&target)) {
      std::cerr << "distance: " << S2Earth::ToMeters(result.distance())
                << std::endl;
      auto coordinate = S2LatLng(query.Project(point, result));
      Projection projection;
      projection.point_on_graph.edge_id.tile_id = tile_id_;
      projection.point_on_graph.edge_id.edge_index = result.shape_id();

      std::cerr << result.edge_id() << "/"
                << spatial_index_.shape(result.shape_id())->num_edges()
                << std::endl;
      // TODO: how do we get the offset ?
      projection.point_on_graph.offset = 0.0;
      projection.coordinate = {coordinate.lng().degrees(),
                               coordinate.lat().degrees()};
      projection.distance_m = S2Earth::ToMeters(result.distance());
      results.push_back(projection);
    }

    return results;
  }

  const auto &edges() const { return header_.edges(); }
  const auto &edges(int index) const { return header_.edges(index); }

  const auto &nodes() const { return header_.nodes(); }
  const auto &nodes(int index) const { return header_.nodes(index); }

  const auto &header() const { return header_; }

private:
  TileId tile_id_;
  tile::Header header_;
  std::unique_ptr<Decoder> decoder_;
  EncodedS2ShapeIndex spatial_index_;
};

class Graph {
public:
  explicit Graph(const std::string &tiles_folder)
      : tiles_folder_(tiles_folder) {}

  std::vector<double> PathDistance(const PointOnGraph &from,
                                   const std::vector<PointOnGraph> &to) {
    constexpr double kMaxDepthMeters = 250.0;
    struct Distance {
      double distance = 0.0;
      EdgeId edge_id;

      bool operator<(const Distance &rhs) const {
        return distance > rhs.distance;
      }
    };

    std::map<EdgeId, size_t> to_find;
    for (size_t index = 0; index < to_find.size(); ++index) {
      to_find[to[index].edge_id] = index;
    }

    std::vector<double> results;
    results.resize(to.size(), std::numeric_limits<double>::max());

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
        to_find.erase(current.edge_id);

        auto finish_edge = GetEdge(to[index].edge_id);
        results[index] =
            current.distance - finish_edge->length() * (1.0 - to[index].offset);
        continue;
      }

      const auto edge = GetEdge(current.edge_id);
      auto target_node =
          GetTargetNode(current.edge_id.tile_id, edge->target_node_id());

      auto adjacent_edge_ids = GetAdjacentEdges(target_node);
      for (const auto &adjacent_edge_id : adjacent_edge_ids) {
        auto edge = GetEdge(adjacent_edge_id);
        if (!edge) {
          continue;
        }

        auto distance = current.distance + edge->length();
        if (distance > kMaxDepthMeters) {
          continue;
        }
        queue.push({distance, adjacent_edge_id});
      }
    }

    return results;
  }

  std::vector<Projection> Project(const Coordinate &coordinate,
                                  double radius_m) {
    // TODO: make coverer class member
    S2RegionCoverer::Options options;
    options.set_fixed_level(11);
    S2RegionCoverer coverer(options);

    S2Cap cap(S2LatLng::FromDegrees(coordinate.lat, coordinate.lon)
                  .Normalized()
                  .ToPoint(),
              S2Earth::MetersToChordAngle(radius_m));
    std::vector<S2CellId> cells;
    coverer.GetCovering(cap, &cells);

    std::vector<Projection> results;
    for (const auto cellId : cells) {
      assert(cellId.level() == 11);

      std::cerr << "cell: " << cellId.id() << std::endl;

      auto tile = GetTile(cellId.id());
      auto tile_results = tile->Project(coordinate, radius_m);
      results.insert(results.end(), tile_results.begin(), tile_results.end());
    }
    return results;
  }

private:
  const tile::Edge *GetEdge(const EdgeId &edge_id) {
    auto tile = GetTile(edge_id.tile_id);
    return tile ? &tile->edges(static_cast<int>(edge_id.edge_index)) : nullptr;
  }

  const tile::Node *GetNode(const NodeId &node_id) {
    auto tile = GetTile(node_id.tile_id);
    return tile ? &tile->nodes(static_cast<int>(node_id.node_index)) : nullptr;
  }

  NodeId GetTargetNode(TileId tile_id, ssize_t node_index) {
    if (node_index >= 0) {
      return {tile_id, static_cast<uint32_t>(node_index)};
    } else {
      // TODO: -1 while encoding?
      auto tile = GetTile(tile_id);
      const auto &neighbor_tile_node = tile->header().neighbour_tile_nodes(
          static_cast<int>(-node_index - 1));
      return {neighbor_tile_node.tile_id(), neighbor_tile_node.node_id()};
    }
  }

  std::vector<EdgeId> GetAdjacentEdges(const NodeId &node_id) {
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

  std::shared_ptr<Tile> GetTile(TileId tile_id) {
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

private:
  std::unordered_map<TileId, std::shared_ptr<Tile>> tiles_;
  std::string tiles_folder_;
};



struct Location {
    double timestamp;
    Coordinate coordinate;
    std::optional<double> bearing = 0.0;
    std::optional<double> speed = 0.0;
    std::optional<double> horizontal_accuracy = 0.0;
};

struct EmissionCost {
public:
    explicit EmissionCost(const Location& location) : location_(location) {}
    double operator()(const Projection& projection) const {
        return 0.0;
    }
private:
    const Location& location_;
};

struct TransitionCost {
public:
    double operator()(const Location& from_location, const Location& to_location, float path_distance) const {
        return 0.0;
    }
}

struct MapMatcher {
public:
    explicit MapMatcher(std::shared_ptr<Graph> graph) : graph_(std::move(graph)) {}
    void Update(const Location& location, state::State& state) {
        auto candidates = graph_->Project(location.coordinate, 100);
        if (candidates.empty()) {
            return;
        }

        EmissionCost emission_cost_computer{location};


        if (state.hmm_states().empty()) {

            for (const auto& candidate: candidates) {
                auto hmm_state = state.add_hmm_states();
                hmm_state->set_sequence_cost(emission_cost(candidate));
            }
            // initialize
        } else {
            for (const auto& candidate: candidates) {
                auto emission_cost = emission_cost_computer(candidate);
                auto transition_cost = 0.f;


                std::vector<PointOnGraph> candidate_points;
                candidate_points.reserve(candidates.size());
                for (const auto& candidate: candidates) {
                    candidate_points.push_back(candidate.point_on_graph);
                }

                for (const auto& hmm_state: state.hmm_states()) {
                    PointOnGraph from_point;
                    from_point.edge_id = {hmm_state.point_on_graph.edge_id().tile_id(), hmm_state.point_on_graph.edge_id().edge_index()};
                    from_point.offset = hmm_state.point_on_graph.offset();

                    auto path_distances = graph_->PathDistance(from_point, candidate_points);


                    // auto hmm_state = state.add_hmm_states();
                    // hmm_state->set_sequence_cost(emission_cost);
                }
            }
            // update
        }
    }
private:
    std::shared_ptr<Graph> graph_;
};


int main(int argc, char **argv) {
  Graph graph(argv[1]);
  std::cerr << graph.Project(Coordinate{13.388860, 52.517037}, 100).size()
            << std::endl;

  return 0;
}