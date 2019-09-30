/**
 * @file Range.h
 *
 * The file defines a template class to represent ranges.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include <algorithm>
#include <type_traits>

/**
 * A template class to represent ranges. It also defines the 13 Allen relations
 */
template<typename T>
STREAMABLE(Range,
{
  /**
   * Constructor.
   * Defines an empty range.
   */
  constexpr Range() : min(T()) COMMA max(T()) {};

  /**
   * Constructor.
   * Defines an empty range.
   * @param minmax A conjoined starting and ending point of the empty range.
   */
  constexpr Range(T minmax) : min(minmax) COMMA max(minmax) {};

  /**
   * Constructor.
   * @param min The minimum of the range.
   * @param max The maximum of the range.
   */
  constexpr Range(T min, T max) : min(min) COMMA max(max) {};

  /** A range between 0 and 1. */
  static constexpr Range<T> ZeroOneRange();

  /** A range between -1 and 1. */
  static constexpr Range<T> OneRange();

  /**
   * The function enlarges the range so that a certain value will be part of it.
   * @param t The value that will be part of the range.
   * @return A reference to the range.
   */
  Range<T>& add(T t)
  {
    if(min > t)
      min = t;
    if(max < t)
      max = t;
    return *this;
  }

  /**
   * The function enlarges the range so that the resulting range also contains another one.
   * @param r The range that also will be part of the range.
   * @return A reference to the range.
   */
  Range<T>& add(const Range<T>& r)
  {
    add(r.min);
    add(r.max);
    return *this;
  }

  /**
   * The function checks whether a certain value is in the range.
   * Note that the function is able to handle circular range, i.e. max < min.
   * @param t The value.
   * @return Is the value inside the range?
   */
  constexpr bool isInside(T t) const {return min <= max ? t >= min && t <= max : t >= min || t <= max;}

  /**
   * The function limits a certain value to the range.
   * Note that the function is not able to handle circular range, i.e. max < min.
   * @param t The value that will be "clipped" to the range.
   * @return The limited value.
   */
  constexpr T limit(T t) const {return t < min ? min : t > max ? max : t;} //sets a limit for a Range

  constexpr T clamped(T t) const { return limit(t); }
  T& clamp(T& t) const { t = clamped(t); return t; }

  template<typename Derived>
  Derived& clamp(Eigen::DenseBase<Derived>& mat) const
  {
    static_assert(std::is_same<typename Eigen::MatrixBase<Derived>::Scalar, T>::value, "Matrix must have the same scalar type as the Range.");
    return mat = mat.derived().unaryExpr([this](T val) { return clamped(val); });
  }

  template<typename Derived>
  Derived clamped(const Eigen::DenseBase<Derived>& mat) const
  {
    static_assert(std::is_same<typename Eigen::MatrixBase<Derived>::Scalar, T>::value, "Matrix must have the same scalar type as the Range.");
    return mat.derived().unaryExpr([this](T val) { return clamped(val); });
  }

  /**
   * The function limits another range to this range.
   * Note that the function is able to handle circular range, i.e. max < min.
   * @param r The range that will be "clipped" to this range.
   * @return The limited value.
   */
  constexpr Range<T> limit(const Range<T>& r) const { return Range<T>(limit(r.min), limit(r.max)); } //sets the limit of a Range

  /**
   * Scales a value t with a range of tRange to this range.
   */
  T scale(T t, const Range<T>& tRange) const;

  /**
   * The function returns the size of the range.
   * @return The difference between the lower limit and the higher limit.
   */
  constexpr T getSize() const {return max - min;}

  /**
   * The function returns the center of the range.
   * @return The center.
   */
  constexpr T getCenter() const {return (max + min) / 2;}

  //!@name The 13 Allen relations
  //!@{
  constexpr bool operator==(const Range<T>& r) const {return min == r.min && max == r.max;}
  constexpr bool operator<(const Range<T>& r) const {return max < r.min;}
  constexpr bool operator>(const Range<T>& r) const {return min > r.max;}
  constexpr bool meets(const Range<T>& r) const {return max == r.min;}
  constexpr bool metBy(const Range<T>& r) const {return min == r.max;}
  constexpr bool overlaps(const Range<T>& r) const {return min < r.min && max < r.max && max > r.min;}
  constexpr bool overlappedBy(const Range<T>& r) const {return min > r.min && max > r.max && min < r.max;}
  constexpr bool starts(const Range<T>& r) const {return min == r.min && max < r.max;}
  constexpr bool startedBy(const Range<T>& r) const {return min == r.min && max > r.max;}
  constexpr bool finishes(const Range<T>& r) const {return max == r.max && min > r.min;}
  constexpr bool finishedBy(const Range<T>& r) const {return max == r.max && min < r.min;}
  constexpr bool during(const Range<T>& r) const {return min > r.min && max < r.max;}
  constexpr bool contains(const Range<T>& r) const {return min < r.min && max > r.max;}
  //!@}

  constexpr bool operator!=(const Range<T>& r) const {return min != r.min || max != r.max;}

  // The size of the intersection of to ranges or 0 if there is no intersection
  constexpr T intersectionSizeWith(const Range<T>& r) const {return std::max(0.f, std::min(max, r.max) - std::max(min, r.min));},

  (T) min,
  (T) max, /**< The limits of the range. */
});

class Angle;

using Rangea = Range<Angle>;
using Rangei = Range<int>;
using Rangef = Range<float>;
using Rangeuc = Range<unsigned char>;

template<typename T>
constexpr Range<T> Range<T>::ZeroOneRange()
{
  return Range<T>(T(0), T(1));
}

template<typename T>
constexpr Range<T> Range<T>::OneRange()
{
  return Range<T>(T(-1), T(1));
}

template<typename T>
T Range<T>::scale(T t, const Range<T>& tRange) const
{
  return limit(((t - tRange.min) / (tRange.max - tRange.min)) * (max - min) + min);
}
