#include "tile.pb.h"
#include <fstream>
#include <iostream>
#include "s2/encoded_s2shape_index.h"
#include "s2/util/coding/coder.h"
#include "s2/s2shapeutil_coding.h"
class Tile {
public:
    explicit Tile(const std::string& path) {
        std::ifstream str(path);
        header_.ParseFromIstream(&str);
        std::cerr << header_.edges_size() << std::endl;
        
        decoder_ = std::make_unique<Decoder>(header_.shape_spatial_index().data(), header_.shape_spatial_index().size());
        spatial_index_.Init(decoder_.get(), s2shapeutil::LazyDecodeShapeFactory(decoder_.get()));

        std::cerr << spatial_index_.num_shape_ids() << std::endl;
    }
private:
    tile::Header header_;
    std::unique_ptr<Decoder> decoder_;
    EncodedS2ShapeIndex spatial_index_;
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

struct Coordinate {
    double lat = 0.0;
    double lon = 0.0;
};

struct Projection {
    PointOnGraph point_on_graph;
    double distance_m = 0.0;
    Coordinate coordinate;
};

class Graph {
public:
    explicit Graph(const std::string& tiles_folder) : tiles_folder_(tiles_folder) {

    }

    std::vector<double> PathDistance(const PointOnGraph& from, const std::vector<PointOnGraph>& to) {
        return {};
    }

    std::vector<Projection> Project(const Coordinate& coordinate, double radius_m) {
        return {};
    }

private:
    std::string tiles_folder_;
};


int main(int argc, char **argv) {
    Tile tile(argv[1]);
    return 0;
}