#pragma once
#include <s2/s2region_coverer.h>
#include <vector>
#include "base/coordinate.hpp"

namespace mama {

class Coverer {
 public:
  Coverer();
  std::vector<S2CellId> GetCovering(const Coordinate& coordinate, double radius_m);

 private:
  S2RegionCoverer coverer_;
};

}  // namespace mama
