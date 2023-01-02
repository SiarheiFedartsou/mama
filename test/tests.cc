#include "mama.hpp"
#include <iostream>
#include <cassert>
#include <limits>

// TODO: we should use a test framework

std::string TilesFolder() {
  assert(getenv("TILES_FOLDER"));
  return getenv("TILES_FOLDER");
}

void TestGraphProjection() {
  mama::Graph graph(TilesFolder());

  assert(graph.Project({0.0, 0.0}, 100).size() == 0);
  assert(graph.Project({7.41795, 43.73247}, 50).size() == 56);

  {
    auto projections = graph.Project({7.41795, 43.73247}, 50);
    assert(std::abs(projections[0].point_on_graph.offset - 0.506949) < 1e-3);
    assert(std::abs(projections[1].point_on_graph.offset - 0.492771) < 1e-3);
    assert(std::abs(projections[0].distance_m - 3.596) < 1e-3);
    assert(projections[0].distance_m == projections[1].distance_m);
  }
}

void TestGraphShortestPath() {
  mama::Graph graph(TilesFolder());

  auto projections = graph.Project({7.41795, 43.73247}, 50);
  // path from exactly the same point
  {
      auto from = projections[0].point_on_graph;
      auto to = from;

      auto path = graph.PathDistance(from, {to}, {250});
      assert(path.size() == 1);
      assert(std::abs(path[0]) <= 1e-10);
  }

  // path to the end of the edge
  {
    auto from = projections[0].point_on_graph;
    auto to = from;
    to.offset = 1.0;

    auto path = graph.PathDistance(from, {to}, {250});
    assert(path.size() == 1);
    assert(std::abs(path[0] - 17.507) < 1e-3);
  }

  // path to the start of the edge
  {
    auto from = projections[0].point_on_graph;
    auto to = from;
    to.offset = 0.0;

    auto path = graph.PathDistance(from, {to}, {250});
    assert(path.size() == 1);
    assert(std::abs(path[0] - 53.016) < 1e-3);
  }


  // path to the start of the edge (shouldn't exist due to max_distance_m)
  {
    auto from = projections[0].point_on_graph;
    auto to = from;
    to.offset = 0.0;

    auto path = graph.PathDistance(from, {to}, {25});
    assert(path.size() == 1);
    assert(path[0] == std::numeric_limits<double>::max());
  }

  // path from one projection to all others
  {
    auto from = projections[0].point_on_graph;
    std::vector<mama::PointOnGraph> to;
    to.reserve(projections.size());
    for (const auto &projection : projections) {
      to.push_back(projection.point_on_graph);
    }
  
    auto path = graph.PathDistance(from, to, {250});
    assert(path.size() == projections.size());
    assert(std::abs(path[0]) <= 1e-10);
  }
}

int main(int argc, char **argv) {
 // TestGraphProjection();
  std::cerr << "TestGraphProjection passed" << std::endl;
  TestGraphShortestPath();
  std::cerr << "TestGraphShortestPath passed" << std::endl;
  return 0;
}