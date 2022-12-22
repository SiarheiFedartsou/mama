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

S2_DEFINE_int32(num_index_points, 10000, "Number of points to index");
S2_DEFINE_int32(num_queries, 10000, "Number of queries");
S2_DEFINE_double(query_radius_km, 100, "Query radius in kilometers");


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

struct Node {
  Coordinate coordinate;
  ObjectID id;
  std::vector<size_t> adjacent_edges;

  std::string getTileId() const {
    S2CellId cellId{S2LatLng::FromDegrees(coordinate.lat(), coordinate.lng())};
    S2CellId tileCellId = cellId.parent(11);
    return tileCellId.ToToken();
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
      auto local_index = getLocalEdgeIndex(adjacent_edge_index, edges[adjacent_edge_index]);
      pbfNode->add_adjacent_edges(local_index);
    }
  }

  size_t getLocalEdgeIndex(size_t global_index, const Edge& edge) {
    auto local_index_itr = global_to_tile_edge_index.find(global_index);
    if (local_index_itr == global_to_tile_edge_index.end()) {
      local_index_itr = global_to_tile_edge_index.emplace(global_index, addEdge(edge)).first;
    }
    return local_index_itr->second;
  }

  ssize_t getLocalNodeIndex(const Node& node) {
    if (node.getTileId() != tile_id) {
      return addNeighbourTileNode(node);
    }
    auto local_index_itr = node_id_to_tile_node_index.find(node.id);
    if (local_index_itr == node_id_to_tile_node_index.end()) {
      local_index_itr = node_id_to_tile_node_index.emplace(node.id, addNode()).first;
    }
    return local_index_itr->second;
  }

  ssize_t addNeighbourTileNode(const Node& node) {
    auto pbfNode = header.add_neighbour_tile_nodes();
    // TODO: hack
    pbfNode->set_tile_id(node.getTileId() + "###" + std::to_string(node.id));
    return header.neighbour_tile_nodes_size() - 1;
  }

  size_t addEdge(const Edge& edge) {
    auto pbfEdge = header.add_edges();
    pbfEdge->set_length(edge.distance);
    return header.edges_size() - 1;
  }

  size_t addNode() {
    auto pbfNode = header.add_nodes();
    return header.nodes_size() - 1;
  }


  std::unordered_map<size_t, size_t> global_to_tile_edge_index;
  std::unordered_map<ObjectID, size_t> node_id_to_tile_node_index;
  
  MutableS2ShapeIndex edge_index;
  std::string tile_id;

  tile::Header header;
};

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " OSMFILE\n";
    return 1;
  }

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



    std::vector<std::unique_ptr<S2Polyline>> polylines;

    std::unordered_map<std::string, TileBuilder> tile_builders;
    for (const auto& [nodeId, node] : builder.nodes) {
      S2CellId cellId{S2LatLng::FromDegrees(node.coordinate.lat(), node.coordinate.lng())};
      S2CellId tileCellId = cellId.parent(11);

      TileBuilder& tile_builder = tile_builders[node.getTileId()];
      tile_builder.tile_id = node.getTileId();
      tile_builder.addNode(node, builder.edges, builder.nodes);

      // for (const auto& edge_index : node.adjacent_edges) {
      //   // auto itr = tile_builder.globalToTileEdgeIndex.find(edge_index);
      //   // if (itr == tile_builder.globalToTileEdgeIndex.end()) {
          
      //   // } else {

      //   // }
      //   // if (globalToTileEdgeIndex.find(edge_index) != globalToTileEdgeIndex.end()) {
      //   //   continue;
      //   // }
      //   // add edge 
      // }

    }
    // for (const auto &edge : builder.edges) {
    //   assert(edge.shape.size() > 1);


    //   S2CellId cellId{S2LatLng::FromDegrees(edge.shape[0].lat(), edge.shape[0].lng())};
    //   S2CellId tileCellId = cellId.parent(11);

    //   std::vector<S2LatLng> latlngs;
    //   for (const auto &node : edge.shape) {
    //     latlngs.emplace_back(node.asS2LatLng());
    //   }

    //   auto polyline = std::make_unique<S2Polyline>(absl::Span<const S2LatLng>{latlngs});
    //   polylines.emplace_back(std::move(polyline));




    //   edge_index.Add(std::make_unique<S2Polyline::Shape>(polylines.back().get()));
    // }

//     std::cerr << edge_index.SpaceUsed() << std::endl;
// //    edge_index.ForceBuild();

//    Encoder encoder;
//   // //s2shapeutil::CompactEncodeTaggedShapes(edge_index, &encoder);
//   // for (S2Shape* shape : edge_index) {
//   //   std::cerr << shape->num_edges() << std::endl;
//   //  // shape->Encode(&encoder);
//   // }
//   s2shapeutil::CompactEncodeTaggedShapes(edge_index, &encoder);
//   edge_index.Encode(&encoder);
//   std::cerr << encoder.length() << std::endl;
    // for (auto& p: u) {
    //   std::cout << p.first << " " << p.second << std::endl;
    // }

  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  // // // Build an index containing random points anywhere on the Earth.
  //  S2PointIndex<int> index;
  // // for (int i = 0; i < absl::GetFlag(FLAGS_num_index_points); ++i) {
  // //   index.Add(S2Testing::RandomPoint(), i);
  // // }

  // // Create a query to search within the given radius of a target point.
  // S2ClosestPointQuery<int> query(&index);
  // query.mutable_options()->set_max_distance(S1Angle::Radians(
  //     S2Earth::KmToRadians(absl::GetFlag(FLAGS_query_radius_km))));

  // // Repeatedly choose a random target point, and count how many index points
  // // // are within the given radius of that point.
  // // int64_t num_found = 0;
  // // // for (int i = 0; i < absl::GetFlag(FLAGS_num_queries); ++i) {
  // // //   S2ClosestPointQuery<int>::PointTarget target(S2Testing::RandomPoint());
  // // //   num_found += query.FindClosestPoints(&target).size();
  // // // }

  // // std::printf("Found %" PRId64 " points in %d queries\n", num_found,
  // //             absl::GetFlag(FLAGS_num_queries));
  // return 0;
}