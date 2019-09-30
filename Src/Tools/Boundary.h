/**
 * @file Boundary.h
 *
 * This file contains the template class Boundary.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Range.h"

/**
 * The template class represents rectangular boundaries.
 */
template<typename T = float> STREAMABLE(Boundary,
{
  Boundary();

  /**
   * Constructor.
   * This constructor allows to specify the minimum and maximum values
   * for type T, e.g. -HUGE_VAL and HUGE_VAL for float. These limits
   * are used to construct the object, so that it can adapt to any boundary later.
   * @param min The minimum value of T.
   * @param max The maximum value of T.
   */
  Boundary(T min, T max);

  /**
   * Constructor.
   * @param x The range in x-direction.
   * @param y The range in y-direction.
   */
  Boundary(const Range<T>& x, const Range<T>& y);

  /**
   * The function enlarges the boundary so that it also includes the specified point.
   * @param p The point.
   */
  void add(const Eigen::Matrix<T, 2, 1>& p)
  {
    x.add(p.x());
    y.add(p.y());
  }

  /**
   * The function enlarges the boundary so that it also includes another boundary.
   * @param b The other boundary.
   */
  void add(const Boundary<T>& b) // add function for adding Boundaries
  {
    x.add(b.x);
    y.add(b.y);
  }

  /**
   * The function checks whether a certain point is enclosed by the boundary
   * @param p The point.
   * @return Lies the point inside the boundary?
   */
  bool isInside(const Eigen::Matrix<T, 2, 1>& p) const {return x.isInside(p.x()) && y.isInside(p.y());}

  /**
   * The function checks whether the boundary is empty
   * @return Is it empty?
   */
  bool isEmpty() const {return x.min > x.max || y.min > y.max;}

  /**
   * Clips the point to the boundary.
   * @param p The point.
   */
  void clip(Eigen::Matrix<T, 2, 1>& p) const
  {
    x.clamp(p.x());
    y.clamp(p.y());
  },

  (Range<T>) x, /**< The range in x-direction. */
  (Range<T>) y, /**< The range in y-direction. */
});

template<typename T> Boundary<T>::Boundary() : x(0, 0), y(0, 0) {}
template<typename T> Boundary<T>::Boundary(T min, T max) : x(max, min), y(max, min) {}
template<typename T> Boundary<T>::Boundary(const Range<T>& x, const Range<T>& y) : x(x), y(y) {}

using Boundaryi = Boundary<int>;
using Boundaryf = Boundary<float>;
