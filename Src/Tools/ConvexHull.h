/**
 * @file Tools/ConvexHull.h
 *
 * This file implements a class for computing the convex hull
 * of a set of points in a plane.
 *
 * @author Alexander Härtl</a>
 */

#pragma once

#include "Tools/Math/Common.h"
#include <vector>
#include <algorithm>
#include "Tools/Math/Vector2.h"

/**
 * A class for computing the convex hull of a set of points in a plane.
 */
class ConvexHull
{
public:
  /**
  * The function computes the convex hull of the given set
  * of points in a plane using the Graham scan algorithm.
  *
  * Note that this implementation currently cannot deal with duplicate points!
  *
  * @param points The set of points.
  */
  template <typename T> static void grahamScan(std::vector<T>& points)
  {
    // if the set of points consists of two points or less, they build the convex hull
    if(points.size() <= 2)
    {
      return;
    }

    // find a point that is part of the convex hull
    typename std::vector<T>::iterator start = min_element(points.begin(), points.end(), lexicographicalComparator<T>);

    // build a comparator object to sort all points by the angle to the given point
    Comparator<T> comp(*start);
    sort(points.begin(), points.end(), comp);

    // temporary stack that initially consists of the first two points of the ordered set of points
    std::vector<T> stack(points.begin(), points.begin() + 2);

    for(typename std::vector<T>::iterator i = points.begin() + 2; i != points.end(); ++i)
    {
      // since collinear points are ordered by distance, collinear points are removed here
      while(stack.size() >= 2 && angleFunction(stack[stack.size() - 2], stack[stack.size() - 1], *i, i->x) >= 0)
      {
        stack.pop_back();
      }
      stack.push_back(*i);
    }
    points = stack;
  }

  /**
  * The function computes the convex hull of the given set
  * of points in a plane using the Jarvis march algorithm.
  *
  * Note that this implementation correctly handles duplicate points.
  *
  * @param points The set of points.
  */
  template <class T> static void jarvisMarch(std::vector<T>& points)
  {
    // if the set of points consists of two points or less, they build the convex hull
    if(points.size() <= 2)
    {
      return;
    }

    // find a point that is part of the convex hull
    typename std::vector<T>::iterator start = min_element(points.begin(), points.end(), lexicographicalComparator<T>);
    typename std::vector<T>::iterator next = start;

    // temporary vector that incrementally holds the convex hull
    std::vector<T> hull;

    do
    {
      hull.push_back(*next);
      Comparator<T> comp(*next);
      next = max_element(points.begin(), points.end(), comp);
    }
    while(start != next);

    points = hull;
  }

private:
  /**
  * This function computes a value that is proportional to the angle between the vectors p0p1 and p0p2.
  * This function is faster and more precise than any triangular function.
  * @param p0 The base point.
  * @param p1 The first end point.
  * @param p2 The other end point.
  * @param voidParam Just to determine the return type.
  * @return A value that is proportional to the angle between the vectors p0p1 and p0p2.
  */
  template <class T, class U> inline static U angleFunction(const T& p0, const T& p1, const T& p2, U voidParam)
  {
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
  }

  /**
  * This function checks if p1 is closer to p0 than p2.
  * @param p0 The base point.
  * @param p1 The first end point.
  * @param p2 The other end point.
  * @return Is p1 closer to p0 than p2?
  */
  template <class T> inline static bool isCloser(const T& p0, const T& p1, const T& p2)
  {
    return (p1 - p0).squareAbs() < (p2 - p0).squareAbs();
  }

  /** This function builds a lexicographical comparator function for a two-vector
  * @param v1 The first point.
  * @param v2 The other point.
  * @return Is v1 lexicographically less than v2?
  */
  template <class T> inline static bool lexicographicalComparator(const T& v1, const T& v2)
  {
    return v1.y < v2.y || (v1.y == v2.y && v1.x < v2.x);
  }

  /**
  * A class to be used as comparator object for sorting points by the angle to a given point.
  */
  template <class T> class Comparator
  {
  private:
    const T& p0; /**< The base point. */
  public:
    /**
    * The only constructor taking the base point as argument.
    * @param p The base point.
    */
    Comparator<T>(const T& p) : p0(p) {}

    /**
    * This function checks if p2 is left of p1, or, if they are collinear, if p1 is closer than p2, both seen from p0
    * @param p1 The first end point.
    * @param p2 The other end point.
    */
    inline bool operator()(const T& p1, const T& p2)
    {
      return angleFunction(p0, p1, p2, p0.x) < 0 || (angleFunction(p0, p1, p2, p0.x) == 0 && isCloser(p0, p1, p2));
    }
  };
};
