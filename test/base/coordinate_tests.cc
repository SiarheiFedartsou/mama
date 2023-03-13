#include "base/coordinate.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace mama {
TEST_CASE("Coordinate.Distance works properly") {
  REQUIRE_THAT((Coordinate{13.388860, 52.517037}.Distance({13.385983, 52.496891})),
               Catch::Matchers::WithinAbs(2250.2807436593, 1e-6));
  REQUIRE_THAT((Coordinate{13.388860, 52.517037}.Distance({13.388860, 52.517037})),
               Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Coordinate.BearingTo works properly") {
  REQUIRE_THAT((Coordinate{13.388860, 52.517037}.BearingTo({13.385983, 52.496891})),
               Catch::Matchers::WithinAbs(184.9789735579, 1e-6));

  REQUIRE_THAT((Coordinate{-96.920341, 32.838261}.BearingTo({-96.920421, 32.838295})),
               Catch::Matchers::WithinAbs(296.7222350862, 1e-6));
}

}  // namespace mama
