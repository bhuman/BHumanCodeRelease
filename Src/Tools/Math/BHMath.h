/**
 * @file Tools/Math/BHMath.h
 *
 * This contains some often used mathematical definitions and functions.
 *
 * @author <a href="mailto:alexists@tzi.de">Alexis Tsogias</a>
 */

#pragma once

#include "Angle.h"
#include <type_traits>

namespace impl
{
  template<typename T, bool IsSigned>
  struct Sgn
  {
    static constexpr int run(const T& x);
  };

  template<typename T>
  struct Sgn<T, false>
  {
    inline static constexpr int run(const T& x)
    {
      return T(0) < x;
    }
  };

  template<typename T>
  struct Sgn<T, true>
  {
    inline static constexpr int run(const T& x)
    {
      return (x > T(0)) - (x < T(0));
    }
  };

  template<typename T, bool IsSigned>
  struct SgnPos
  {
    static constexpr int run(const T& x);
  };

  template<typename T>
  struct SgnPos<T, false>
  {
    inline static constexpr int run(const T& x)
    {
      return 1;
    }
  };

  template<typename T>
  struct SgnPos<T, true>
  {
    inline static constexpr int run(const T& x)
    {
      return (x >= T(0)) - (x < T(0));
    }
  };
}

/**
 * Returns the sign of a value (-1, 0, or 1).
 * @param x The value.
 * @return The sign of x.
 */
template<typename T>
constexpr int sgn(const T& x)
{
  return impl::Sgn<T, std::is_signed<T>::value>::run(x);
}

template<>
constexpr int sgn<Angle>(const Angle& x)
{
  return sgn(static_cast<float>(x));
}

/**
 * Returns the sign of a value (-1 or 1). 0 is considered to be positive.
 * @param x The value.
 * @return The sign of x.
 */
template<typename T>
constexpr int sgnPos(const T& x)
{
  return impl::SgnPos<T, std::is_signed<T>::value>::run(x);
}

template<>
constexpr int sgnPos<Angle>(const Angle& x)
{
  return sgnPos(static_cast<float>(x));
}

/**
 * Returns the sign of a value (-1 or 1). 0 is considered to be negative.
 * @param x The value.
 * @return The sign of x.
 */
template<typename T>
constexpr int sgnNeg(const T& x)
{
  return (x > T(0)) - (x <= T(0));
}

/**
 * Calculates the square of a value.
 * @param a The value.
 * @return The square of \c a.
 */
template<class V>
constexpr V sqr(const V& a) { return a * a; }

/**
 * Defines a macro that returns a value, in which the specified bit is set.
 * @param t The bit to set. Must be in the range of [0 .. 31].
 * @return A 32 bit value in which the bit is set.
 */
template<typename T>
constexpr unsigned bit(T t)
{
  return 1 << static_cast<unsigned>(t);
}

/**
 * Checks, if a given value is between two others
 * @param value The value
 * @param a The lower bound
 * @param b The upper bound
 * @return The square of \c a.
 */
template<typename T>
bool between(const T value, const T a, const T b)
{
  return value >= (a < b ? a : b) && value <= (a > b ? a : b);
}

/**
 * Clips a value to a range
 * @param val The value
 * @param min The lower bound
 * @param max The upper bound
 * @return The clipped value
 */
template<typename T>
T clip(const T val, const T min, const T max)
{
  if(val <= min)
    return min;
  else if(val >= max)
    return max;
  else
    return val;
}
