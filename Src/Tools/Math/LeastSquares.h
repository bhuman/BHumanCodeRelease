/**
 * @file LeastSquares.h
 *
 * Contains classes and functions for fitting models to data points using linear
 * least squares fitting.
 *
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Math/MeanCalculator.h"

namespace LeastSquares
{
  /**
   * A class that fits a line through the given points by minimizing the squared
   * residuals of points relative to the line.
   */
  class LineFitter : public MeanCalculator<Eigen::Matrix<float, 2, 3>, Vector2f>
  {
  public:
    LineFitter() : MeanCalculator<Eigen::Matrix<float, 2, 3>, Vector2f>([](const Vector2f& v)
    {
      Eigen::Matrix<float, 2, 3> m;
      m.col(0) = v;
      m.col(1) = v.cwiseProduct(v);
      m(0, 2) = v.x() * v.y();
      return m;
    }) {}

    /**
     * Fits a line through all the points that were added to the LineFitter.
     *
     * @param n0 reference to a variable to be filled with the normal of the line
     * @param d reference to a variable to be filled with the distance of the line
     */
    bool fit(Vector2f& n0, float& d) const;
  };

  /**
   * A class that fits a circle through the given points by minimizing the
   * squared residuals of points relative to the circle.
   */
  class CircleFitter : public MeanCalculator<Eigen::Matrix<float, 2, 4>, Vector2f>
  {
  public:
    CircleFitter() : MeanCalculator<Eigen::Matrix<float, 2, 4>, Vector2f>([](const Vector2f& v)
    {
      Eigen::Matrix<float, 2, 4> m;
      const Vector2f v2(v.cwiseProduct(v));

      m.col(0) = v;
      m.col(1) = v2;
      m.col(2) = v.cwiseProduct(v2) + v.cwiseProduct(v2.reverse());
      m(0, 3) = v.x() * v.y();
      return m;
    }) {}

    /**
     * Fits a circle through all the points that were added to the CircleFitter.
     *
     * @param center reference to a variable to be filled with the center of the circle
     * @param radius reference to a variable to be filled with the radius of the circle
     */
    bool fit(Vector2f& center, float& radius) const;
  };

  /**
   * Fits a line through the given points by minimizing the squared residuals of
   * points relative to the line.
   *
   * @param begin iterator to the beginning of the points (inclusive)
   * @param end iterator to the end of the points (exclusive)
   * @param n0 reference to a variable to be filled with the normal of the line
   * @param d reference to a variable to be filled with the distance of the line
   */
  template<typename IteratorType>
  static inline bool fitLine(const IteratorType& begin, const IteratorType& end, Vector2f& n0, float& d)
  {
    LineFitter fitter;
    fitter.add(begin, end);
    return fitter.fit(n0, d);
  }

  /**
   * Fits a line through the given points by minimizing the squared residuals of
   * points relative to the line.
   *
   * @param points vector of points to be fitted to a line
   * @param n0 reference to a variable to be filled with the normal of the line
   * @param d reference to a variable to be filled with the distance of the line
   */
  template<typename ContainerType>
  static inline bool fitLine(const ContainerType& points, Vector2f& n0, float& d)
  {
    return fitLine(points.cbegin(), points.cend(), n0, d);
  }

  /**
   * Fits a circle through the given points by minimizing the squared residuals
   * of points relative to the circle.
   *
   * @param begin iterator to the beginning of the points (inclusive)
   * @param end iterator to the end of the points (exclusive)
   * @param center reference to a variable to be filled with the center of the circle
   * @param radius reference to a variable to be filled with the radius of the circle
   */
  template<typename IteratorType>
  static inline bool fitCircle(const IteratorType& begin, const IteratorType& end, Vector2f& center, float& radius)
  {
    CircleFitter fitter;
    fitter.add(begin, end);
    return fitter.fit(center, radius);
  }

  /**
   * Fits a circle through the given points by minimizing the squared residuals
   * of points relative to the circle.
   *
   * @param points vector of points to be fitted to a circle
   * @param center reference to a variable to be filled with the center of the circle
   * @param radius reference to a variable to be filled with the radius of the circle
   */
  template<typename ContainerType>
  static inline bool fitCircle(const ContainerType& points, Vector2f& center, float& radius)
  {
    return fitCircle(points.cbegin(), points.cend(), center, radius);
  }
};
