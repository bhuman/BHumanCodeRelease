#pragma once

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
}
