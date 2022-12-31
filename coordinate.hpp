#pragma once

#include <s2/s2latlng.h>

namespace mama {

struct Coordinate {
  double x{};
  double y{};

  double lng() const { return x; }

  double lat() const { return y; }

  S2LatLng AsS2LatLng() const { return S2LatLng::FromDegrees(y, x); }
};

} // namespace mama