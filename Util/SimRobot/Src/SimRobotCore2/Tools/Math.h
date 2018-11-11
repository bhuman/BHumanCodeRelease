/**
 * @file Tools/Math.h
 * A collection of general purpose math functions.
 * @author Thomas Röfer
 */

#pragma once

#include <cmath>

/**
 * Normalizes an angle to the range [-pi .. pi[.
 * @tparam T The type of the angle (usually float or double).
 * @param angle The angle that is normalized.
 * @return The angle normalized to the range [-pi .. pi[.
 */
template<typename T> T normalize(T angle)
{
  if(angle < -T(M_PI) || angle > T(M_PI))
  {
    angle = angle - static_cast<int>(angle / T(2 * M_PI)) * T(2 * M_PI);
    if(angle >= T(M_PI))
      angle = T(angle - T(2 * M_PI));
    else if(angle < -T(M_PI))
      angle = T(angle + T(2 * M_PI));
  }
  return angle;
}
