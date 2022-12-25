#include "s2/encoded_s2shape_index.h"
#include "s2/s2closest_edge_query.h"

#include "s2/s2cap.h"
#include "s2/s2earth.h"
#include "s2/s2latlng.h"
#include "s2/s2region_coverer.h"
#include "s2/s2shapeutil_coding.h"
#include "s2/util/coding/coder.h"
#include "tile.pb.h"
#include <fstream>
#include <iostream>
#include <filesystem>

struct Coordinate {
  double lon = 0.0;
  double lat = 0.0;
};

using TileId = uint64_t;

struct EdgeId {
  TileId tile_id;
  uint32_t edge_index = 0;
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

  std::vector<Projection> Project(const Coordinate &coordinate, double radius_m) {
    S2ClosestEdgeQuery query(&spatial_index_);
   // query.mutable_options()->set_max_results(5);
    query.mutable_options()->set_max_distance(
        S2Earth::MetersToChordAngle(radius_m));
    auto point = S2LatLng::FromDegrees(coordinate.lat, coordinate.lon)
                  .Normalized()
                  .ToPoint();
    S2ClosestEdgeQuery::PointTarget target(point);

    std::vector<Projection> results;
    for (const auto& result : query.FindClosestEdges(&target)) {
        std::cerr << "distance: " << S2Earth::ToMeters(result.distance()) << std::endl;
        auto coordinate = S2LatLng(query.Project(point, result));
        Projection projection;
        projection.point_on_graph.edge_id.tile_id = tile_id_;
        projection.point_on_graph.edge_id.edge_index = result.shape_id();
        // TODO: how do we get the offset ? 
        projection.point_on_graph.offset = 0.0;
        projection.coordinate = {coordinate.lng().degrees(), coordinate.lat().degrees()};
        projection.distance_m = S2Earth::ToMeters(result.distance());
        results.push_back(projection);
    }

    return results;

  }

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
    return {};
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

int main(int argc, char **argv) {
  Graph graph(argv[1]);
  std::cerr << graph.Project(Coordinate{13.388860, 52.517037}, 100).size() << std::endl;
  

  return 0;
}