#include "graph/graph.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace mama {
namespace {
std::string TilesFolder() {
  REQUIRE(getenv("TILES_FOLDER"));
  return getenv("TILES_FOLDER");
}
} // namespace

TEST_CASE("Project properly finds projections on graph") {
  Graph graph(TilesFolder());

  REQUIRE(graph.Project({0.0, 0.0}, 100).size() == 0);
  REQUIRE(graph.Project({7.41795, 43.73247}, 50).size() == 56);

  {
    auto projections = graph.Project({7.41795, 43.73247}, 50);
    REQUIRE_THAT(projections[0].point_on_graph.offset,
                 Catch::Matchers::WithinAbs(0.506949, 1e-3));
    REQUIRE_THAT(projections[1].point_on_graph.offset,
                 Catch::Matchers::WithinAbs(0.492771, 1e-3));
    REQUIRE_THAT(projections[0].distance_m,
                 Catch::Matchers::WithinAbs(3.596, 1e-3));
    REQUIRE_THAT(projections[1].distance_m,
                 Catch::Matchers::WithinAbs(3.596, 1e-3));

    REQUIRE_THAT(projections[0].bearing_deg,
                 Catch::Matchers::WithinAbs(27.6450, 1e-3));
    REQUIRE_THAT(projections[1].bearing_deg,
                 Catch::Matchers::WithinAbs(207.6451, 1e-3));

    REQUIRE(projections[0].distance_m == projections[1].distance_m);

    for (const auto &p : projections) {
      REQUIRE(((0 <= p.bearing_deg) && (p.bearing_deg < 360.0)));
    }
  }
}

TEST_CASE("PathDistance properly finds shortest path") {
  Graph graph(TilesFolder());

  auto projections = graph.Project({7.41795, 43.73247}, 50);
  // path from exactly the same point
  {
    auto from = projections[0].point_on_graph;
    auto to = from;

    auto path = graph.PathDistance(from, {to}, {250});
    REQUIRE(path.size() == 1);
    REQUIRE_THAT(path[0], Catch::Matchers::WithinAbs(0.0, 1e-10));
  }

  // path to the end of the edge
  {
    auto from = projections[0].point_on_graph;
    auto to = from;
    to.offset = 1.0;

    auto path = graph.PathDistance(from, {to}, {250});
    REQUIRE(path.size() == 1);
    REQUIRE_THAT(path[0], Catch::Matchers::WithinAbs(17.497, 1e-3));
  }

  // path to the start of the edge
  {
    auto from = projections[0].point_on_graph;
    auto to = from;
    to.offset = 0.0;

    auto path = graph.PathDistance(from, {to}, {250});
    REQUIRE(path.size() == 1);
    REQUIRE_THAT(path[0], Catch::Matchers::WithinAbs(52.996, 1e-3));
  }

  // path to the start of the edge (shouldn't exist due to max_distance_m)
  {
    auto from = projections[0].point_on_graph;
    auto to = from;
    to.offset = 0.0;

    auto path = graph.PathDistance(from, {to}, {25});
    REQUIRE(path.size() == 1);
    REQUIRE(path[0] == std::numeric_limits<double>::max());
  }

  // path from one projection to all others
  {
    auto from = projections[0].point_on_graph;
    std::vector<PointOnGraph> to;
    to.reserve(projections.size());
    for (const auto &projection : projections) {
      to.push_back(projection.point_on_graph);
    }

    auto path = graph.PathDistance(from, to, {250});
    REQUIRE(path.size() == projections.size());
    REQUIRE_THAT(path[0], Catch::Matchers::WithinAbs(0.0, 1e-10));
  }
}

} // namespace mama
