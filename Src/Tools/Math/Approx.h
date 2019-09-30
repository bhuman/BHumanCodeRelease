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
    using std::abs;
    return abs(a) < prec;
  }

  template<typename T>
  bool isEqual(T a, T b, T prec = std::numeric_limits<T>::epsilon())
  {
    using std::abs;
    using std::max;
    const T diff = abs(a - b);
    return diff < prec || diff < prec * max(abs(a), abs(b));
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

  /**
   * An integer-math only approximation of atan2.
   * Modified version of the Q15-implementation from http://geekshavefeelings.com/posts/fixed-point-atan2.
   *
   * @param y y coordinate as a short in the range [-16384, 16384)
   * @param x x coordinate as a short in the range [-16384, 16384)
   * @return atan2 of y and x as an unsigned short in the range [0,65536)
   */
  inline unsigned short atan2(const short y, const short x)
  {
    short quotient;
    unsigned short offset;
    if(std::abs(x) > std::abs(y))
    {
      quotient = static_cast<short>((y << 14) / x);
      offset = x > 0 ? 0 : 32768;
    }
    else
    {
      quotient = x == 0 ? 0 : static_cast<short>(-1 * (x << 14) / y);
      offset = y > 0 ? 16384 : 49152;
    }
    return offset + static_cast<unsigned short>((static_cast<int>(11039 - ((5695 * static_cast<int>(std::abs(quotient))) >> 15)) * static_cast<int>(quotient)) >> 15);
  }
}
