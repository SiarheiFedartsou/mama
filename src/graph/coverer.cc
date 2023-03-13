#include "coverer.hpp"
#include "tile_level.hpp"

#include <s2/s2cap.h>
#include <s2/s2earth.h>

namespace mama {

namespace {
S2RegionCoverer::Options MakeOptions() {
  S2RegionCoverer::Options options;
  options.set_fixed_level(graph::kTileLevel);
  return options;
}
}  // namespace

Coverer::Coverer() : coverer_(MakeOptions()) {}

std::vector<S2CellId> Coverer::GetCovering(const Coordinate& coordinate, double radius_m) {
  S2Cap cap(coordinate.AsS2LatLng().Normalized().ToPoint(), S2Earth::MetersToChordAngle(radius_m));
  std::vector<S2CellId> cells;
  coverer_.GetCovering(cap, &cells);
  return cells;
}

}  // namespace mama
