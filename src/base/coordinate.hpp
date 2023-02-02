#pragma once

#include <s2/s2earth.h>
#include <s2/s2latlng.h>

#include <mapbox/cheap_ruler.hpp>

namespace mama {

struct Coordinate {
  double x{};
  double y{};

  double lng() const { return x; }

  double lat() const { return y; }

  static Coordinate FromS2LatLng(const S2LatLng &latlng) {
    const auto& normalized = latlng.Normalized();
    return Coordinate{normalized.lng().degrees(), normalized.lat().degrees()};
  }

  S2LatLng AsS2LatLng() const { return S2LatLng::FromDegrees(y, x); }

  // returns distance to `coordinate` in meters
  double Distance(const Coordinate &coordinate) const {
    // TODO: we can cache rulers by latitude and reuse them
    mapbox::cheap_ruler::CheapRuler ruler{lat(), mapbox::cheap_ruler::CheapRuler::Meters};

    return ruler.distance({x, y}, {coordinate.x, coordinate.y});
  }

  // returns bearing in direction of `coordinate` in degrees
  double BearingTo(const Coordinate &coordinate) const {
    mapbox::cheap_ruler::CheapRuler ruler{lat(), mapbox::cheap_ruler::CheapRuler::Meters};

    auto bearing = ruler.bearing({x, y}, {coordinate.x, coordinate.y});
    if (bearing < 0) {
      bearing += 360;
    }
    assert(bearing >= 0.0 && bearing <= 360.0);
    return bearing;
  }
};

} // namespace mama