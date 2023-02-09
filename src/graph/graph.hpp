#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include "base/coordinate.hpp"
#include <functional>
namespace mama {

namespace tile {
class Edge;
class Node;
} // namespace tile

using TileId = uint64_t;

struct EdgeId {
  TileId tile_id = 0;
  uint32_t edge_index = 0;

  bool operator==(const EdgeId &other) const {
    return tile_id == other.tile_id && edge_index == other.edge_index;
  }
};

// TODO: define hash function for EdgeId
struct EdgeIdHasher
{
  std::size_t operator()(const EdgeId& e) const
  {
    return std::hash<TileId>()(e.tile_id) ^ std::hash<uint32_t>()(e.edge_index);
  }
};

struct NodeId {
  TileId tile_id = 0;
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

class Tile;

class Graph {
public:
  struct PathOptions {
    double max_distance_m = 250.0;
  };

  explicit Graph(const std::string &tiles_folder);

  std::vector<double> PathDistance(const PointOnGraph &from,
                                   const std::vector<PointOnGraph> &to,
                                   const PathOptions &options);
  std::vector<Projection> Project(const Coordinate &coordinate,
                                  double radius_m);

private:
  const tile::Edge *GetEdge(const EdgeId &edge_id);

  const tile::Node *GetNode(const NodeId &node_id);
  std::optional<NodeId> GetTargetNode(TileId tile_id, ssize_t node_index);

  std::shared_ptr<Tile> GetTile(TileId tile_id);

private:


  std::unordered_map<TileId, std::shared_ptr<Tile>> tiles_;
  std::string tiles_folder_;
};

} // namespace mama
