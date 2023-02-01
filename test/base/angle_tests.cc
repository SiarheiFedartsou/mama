#include "base/angle.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace mama {
TEST_CASE("AngleDiff works properly") {
  REQUIRE_THAT(AngleDiff(0.0, 0.0), Catch::Matchers::WithinAbs(0.0, 1e-6));
  REQUIRE_THAT(AngleDiff(0.0, 180.0), Catch::Matchers::WithinAbs(180.0, 1e-6));
  REQUIRE_THAT(AngleDiff(0.0, 360.0), Catch::Matchers::WithinAbs(0.0, 1e-6));
  REQUIRE_THAT(AngleDiff(350.0, 32.0), Catch::Matchers::WithinAbs(42.0, 1e-6));
  REQUIRE_THAT(AngleDiff(32.0, 350.0), Catch::Matchers::WithinAbs(42.0, 1e-6));
  REQUIRE_THAT(AngleDiff(710.0, 32.0), Catch::Matchers::WithinAbs(42.0, 1e-6));
  REQUIRE_THAT(AngleDiff(710.0, 392.0), Catch::Matchers::WithinAbs(42.0, 1e-6));
  REQUIRE_THAT(AngleDiff(32.0, 710.0), Catch::Matchers::WithinAbs(42.0, 1e-6));
  REQUIRE_THAT(AngleDiff(392.0, 710.0), Catch::Matchers::WithinAbs(42.0, 1e-6));
}

} // namespace mama
