#pragma once

#include <vector>

/**
 * Calculates the absolute deviation of v1 and v2.
 *
 * @param v1 first value
 * @param v2 second value
 * @return absolute deviation of v1 and v2 (positive)
 */
template<typename T> inline constexpr T getAbsoluteDeviation(const T v1, const T v2)
{
  return v1 < v2 ? v2 - v1 : v1 - v2;
}

/**
 * Calculates the relative deviation of v1 and v2.
 * Result is only meaningful if sgn(v1) == sgn(v2).
 *
 * @param v1 first value
 * @param v2 second value
 * @return relative deviation of v1 and v2 (0..1); 0 means v1 == v2, 0.5 means v1 == v2/2 or v2 == v1/2
 */
template<typename T> inline constexpr float getRelativeDeviation(const T v1, const T v2)
{
  return 1.f - (v1 < v2 ? static_cast<float>(v1) / static_cast<float>(v2) : static_cast<float>(v2) / static_cast<float>(v1));
}
