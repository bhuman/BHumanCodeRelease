/**
 * @file Range.h
 *
 * The file defines a template class to represent ranges.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
 * A template class to represent ranges. It also defines the 13 Allen relations
 */
template<typename T> STREAMABLE(Range,
{
  /**
   * Constructor.
   * Defines an empty range.
   */
  Range();

  /**
   * Constructor.
   * Defines an empty range.
   * @param minmax A conjoined starting and ending point of the empty range.
   */
  Range(T minmax);

  /**
   * Constructor.
   * @param min The minimum of the range.
   * @param max The maximum of the range.
   */
  Range(T min, T max);

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

  /** A range between 0 and 1. */
  static const Range<T>& ZeroOneRange();

  /** A range between -1 and 1. */
  static const Range<T>& OneRange();

  /**
   * The function checks whether a certain value is in the range.
   * Note that the function is able to handle circular range, i.e. max < min.
   * @param t The value.
   * @return Is the value inside the range?
   */
  bool isInside(T t) const {return min <= max ? t >= min && t <= max : t >= min || t <= max;}

  /**
   * The function limits a certain value to the range.
   * Note that the function is not able to handle circular range, i.e. max < min.
   * @param t The value that will be "clipped" to the range.
   * @return The limited value.
   */
  T limit(T t) const {return t < min ? min : t > max ? max : t;} //sets a limit for a Range

  /**
   * The function limits another range to this range.
   * Note that the function is able to handle circular range, i.e. max < min.
   * @param r The range that will be "clipped" to this range.
   * @return The limited value.
   */
  Range<T> limit(const Range<T>& r) const {return Range<T>(limit(r.min), limit(r.max));} //sets the limit of a Range

  /**
   * The function returns the size of the range.
   * @return The difference between the lower limit and the higher limit.
   */
  T getSize() const {return max - min;}

  /**
   * The function returns the center of the range.
   * @return The center.
   */
  T getCenter() const {return (max + min) / 2;}

  //!@name The 13 Allen relations
  //!@{
  bool operator==(const Range<T>& r) const {return min == r.min && max == r.max;}
  bool operator<(const Range<T>& r) const {return max < r.min;}
  bool operator>(const Range<T>& r) const {return min > r.max;}
  bool meets(const Range<T>& r) const {return max == r.min;}
  bool metBy(const Range<T>& r) const {return min == r.max;}
  bool overlaps(const Range<T>& r) const {return min < r.min && max < r.max && max > r.min;}
  bool overlappedBy(const Range<T>& r) const {return min > r.min && max > r.max && min < r.max;}
  bool starts(const Range<T>& r) const {return min == r.min && max < r.max;}
  bool startedBy(const Range<T>& r) const {return min == r.min && max > r.max;}
  bool finishes(const Range<T>& r) const {return max == r.max && min > r.min;}
  bool finishedBy(const Range<T>& r) const {return max == r.max && min < r.min;}
  bool during(const Range<T>& r) const {return min > r.min && max < r.max;}
  bool contains(const Range<T>& r) const {return min < r.min && max > r.max;}
  //!@}

  // The size of the intersection of to ranges or 0 if there is no intersection
  T intersectionSizeWith(const Range<T>& r) const {return std::max(0.f, std::min(max, r.max) - std::max(min, r.min));},

  (T) min,
  (T) max, /**< The limits of the range. */
});

template<typename T> Range<T>::Range() : min(T()), max(T()) {}
template<typename T> Range<T>::Range(T minmax) : min(minmax), max(minmax) {}
template<typename T> Range<T>::Range(T min, T max) : min(min), max(max) {}

template<typename T>
const Range<T>& Range<T>::ZeroOneRange()
{
  static Range<T> range(0, 1);
  return range;
}

template<typename T>
const Range<T>& Range<T>::OneRange()
{
  static Range<T> range(-1, 1);
  return range;
}

using Rangei = Range<int>;
using Rangef = Range<float>;
