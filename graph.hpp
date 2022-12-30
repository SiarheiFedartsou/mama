#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace mama {

namespace tile {
class Edge;
class Node;
} // namespace tile

using TileId = uint64_t;

struct Coordinate {
  double lon = 0.0;
  double lat = 0.0;
};

struct EdgeId {
  TileId tile_id;
  uint32_t edge_index = 0;
};

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

class Tile;

class Graph {
public:
  explicit Graph(const std::string &tiles_folder);

  std::vector<double> PathDistance(const PointOnGraph &from,
                                   const std::vector<PointOnGraph> &to);
  std::vector<Projection> Project(const Coordinate &coordinate,
                                  double radius_m);

private:
  const tile::Edge *GetEdge(const EdgeId &edge_id);

  const tile::Node *GetNode(const NodeId &node_id);
  NodeId GetTargetNode(TileId tile_id, ssize_t node_index);
  std::vector<EdgeId> GetAdjacentEdges(const NodeId &node_id);

  std::shared_ptr<Tile> GetTile(TileId tile_id);

private:
  std::unordered_map<TileId, std::shared_ptr<Tile>> tiles_;
  std::string tiles_folder_;
};

} // namespace mama
