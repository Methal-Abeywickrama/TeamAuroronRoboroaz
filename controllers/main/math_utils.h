#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <algorithm>
#include <cmath>

namespace MathUtil {

class MathUtils {
public:
  // Static helper functions

  // Example: Normalize angle to [-PI, PI]
  static double normalizeAngle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  // Example: Clamp value
  static double clamp(double value, double min, double max) {
    if (value < min)
      return min;
    if (value > max)
      return max;
    return value;
  }
};
} // namespace MathUtil

#endif // MATH_UTILS_H
