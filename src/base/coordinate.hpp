#pragma once

#include <s2/s2earth.h>
#include <s2/s2latlng.h>

namespace mama {

struct Coordinate {
  double x{};
  double y{};

  double lng() const { return x; }

  double lat() const { return y; }

  S2LatLng AsS2LatLng() const { return S2LatLng::FromDegrees(y, x); }

  // TODO: use cheap ruler
  double Distance(const Coordinate &coordinate) const {
    return S2Earth::ToMeters(AsS2LatLng().Normalized().GetDistance(
        coordinate.AsS2LatLng().Normalized()));
  }

  double BearingTo(const Coordinate &coordinate) const {

    auto radians = [](double degrees) { return degrees * M_PI / 180.0; };
    auto degrees = [](double radians) { return radians * 180.0 / M_PI; };

    double teta1 = radians(lat());
    double teta2 = radians(coordinate.lat());
    double delta1 = radians(coordinate.lat() - lat());
    double delta2 = radians(coordinate.lng() - lng());

    double y = std::sin(delta2) * std::cos(teta2);
    double x = std::cos(teta1) * std::sin(teta2) -
               std::sin(teta1) * std::cos(teta2) * std::cos(delta2);
    double bearing = std::atan2(y, x);
    bearing = degrees(bearing);
    bearing = (((int)bearing + 360) % 360);
    return bearing;
  }
};

} // namespace mama