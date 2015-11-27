#pragma once

#include "BHMath.h"
#include "Constants.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Approx
{
  template<typename T>
  bool isZero(T a, T prec = std::numeric_limits<T>::epsilon())
  {
    return std::abs(a) < prec;
  }

  template<typename T>
  bool isEqual(T a, T b, T prec = std::numeric_limits<T>::epsilon())
  {
    const T diff = std::abs(a - b);
    return diff < prec || diff < prec * std::max(std::abs(a), std::abs(b));
  }

  /**
   * An approximation of atan2 with an error < 0.005f.
   * 3-5x times faster than atan2 from cmath
   */
  inline float atan2(float y, float x)
  {
    if(x == 0.f)
      return sgn(y) * pi_2;
    const float z = y / x;
    if(std::abs(z) < 1.f)
    {
      const float atan = z / (1.f + 0.28f * z * z);
      if(x < 0.f)
      {
        if(y < 0.f)
          return atan - pi;
        else
          return atan + pi;
      }
      else
        return atan;
    }
    else
    {
      const float atan = pi_2 - z / (z * z + 0.28f);
      if(y < 0.f)
        return atan - pi;
      else
        return atan;
    }
  }
}
