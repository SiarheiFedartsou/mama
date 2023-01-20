#pragma once

#include <cmath>

namespace mama {

inline double NormalizeAngle(double angle) {
  angle = std::fmod(angle, 360.0);
  if (angle < 0.0) {
    angle += 360.0;
  }
  return angle;
}

inline double AngleDiff(double angle1, double angle2) {
  // normalize angles
  angle1 = NormalizeAngle(angle1);
    angle2 = NormalizeAngle(angle2);
  double diff = angle1 - angle2;
  if (diff > 180.0) {
    diff -= 360.0;
  } else if (diff < -180.0) {
    diff += 360.0;
  }
  return std::abs(diff);
}

} // namespace mama