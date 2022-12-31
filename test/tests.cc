#include "mama.hpp"
#include <iostream>
#include <cassert>

// TODO: we should use a test framework

std::string TilesFolder() {
  assert(getenv("TILES_FOLDER"));
  return getenv("TILES_FOLDER");
}

void TestGraphProjection() {
  mama::Graph graph(TilesFolder());

  assert(graph.Project({0.0, 0.0}, 100).size() == 0);
  assert(graph.Project({7.41795, 43.73247}, 50).size() == 56);
}

void TestGraphShortestPath() {
  mama::Graph graph(TilesFolder());

  auto projections = graph.Project({7.41795, 43.73247}, 50);
  // path from exactly the same point
  {
      auto from = projections[0];
      auto to = from;

      auto path = graph.PathDistance(from.point_on_graph, {to.point_on_graph});
      assert(path.size() == 1);
      assert(std::abs(path[0]) <= 1e-10);
  }
}

int main(int argc, char **argv) {
  TestGraphProjection();
  std::cerr << "TestGraphProjection passed" << std::endl;
  TestGraphShortestPath();
  std::cerr << "TestGraphShortestPath passed" << std::endl;
  return 0;
}