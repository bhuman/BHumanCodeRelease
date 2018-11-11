/**
 * Contains a class for computing a sum as exact as possible using Neumaier's
 * method.
 *
 * @author Felix Thielke
 */

#pragma once

#include <type_traits>
#include <cmath>

/**
 * A class for computing a floating point sum as exact as possible using
 * Neumaier's method.
 * see https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
 */
template<typename T = float>
class NeumaierSum
{
  static_assert(std::is_floating_point<T>::value, "");

  T sum;
  T c;

public:
  NeumaierSum() : sum(0), c(0) {}

  /**
   * Initializes the sum with the given value.
   */
  NeumaierSum(const T x) : sum(x), c(0) {}

  /**
   * Adds the given value to the sum.
   */
  inline NeumaierSum& operator+=(const T x)
  {
    const T t = sum + x;

    if(std::abs(sum) >= std::abs(x))
      c += (sum - t) + x;
    else
      c += (x - t) + sum;

    sum = t;

    return *this;
  }

  /**
   * Returns the calculated sum.
   */
  inline operator T() const
  {
    return sum + c;
  }
};
