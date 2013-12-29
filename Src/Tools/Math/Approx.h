
#pragma once

#include <cmath>

 /**
 * An approximation of atan2 with an error < 0.005f.
 * 3-5x times faster than atan2 from cmath
 */
inline float approxAtan2(float y, float x)
{
  if(x == 0.f)
  {
    if(y > 0.0f) return 1.5707963f;
    if(y == 0.0f) return 0.0f;
    return -1.5707963f;
  }
  float atan;
  float z = y / x;
  if(fabs(z) < 1.f)
  {
    atan = z / (1.f + 0.28f * z * z);
    if(x < 0.0f)
    {
      if(y < 0.0f) return atan - 3.14159265f;
      return atan + 3.14159265f;
    }
  }
  else
  {
    atan = 1.5707963f - z / (z * z + 0.28f);
    if(y < 0.f) return atan - 3.14159265f;
  }
  return atan;
}
