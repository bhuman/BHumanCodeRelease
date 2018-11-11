#pragma once

#include "Tools/Math/Eigen.h"
#include <vector>

namespace LeastSquares
{
  /**
   * Fits a line through the given points using least-squares fitting.
   *
   * @param points vector of points to be fitted to a line
   * @param center reference to a variable to be filled with the normal of the line
   * @param radius reference to a variable to be filled with the distance of the line
   */
  void fitLine(const std::vector<Vector2f>& points, Vector2f& n0, float& d);

  /**
   * Fits a line through the given points using least-squares fitting.
   *
   * @param itBegin iterator to the beginning of the points (inclusive)
   * @param itEnd iterator to the end of the points (exclusive)
   * @param center reference to a variable to be filled with the normal of the line
   * @param radius reference to a variable to be filled with the distance of the line
   */
  void fitLine(const std::vector<Vector2f>::const_iterator& itBegin, const std::vector<Vector2f>::const_iterator& itEnd, Vector2f& n0, float& d);

  /**
   * Fits a line through the given points using least-squares fitting.
   *
   * @param points vector of points to be fitted to a line
   * @param center reference to a variable to be filled with the normal of the line
   * @param radius reference to a variable to be filled with the distance of the line
   * @return average deviation of the points to the fitted line
   */
  float fitLineWithError(const std::vector<Vector2f>& points, Vector2f& n0, float& d);

  /**
   * Fits a line through the given points using least-squares fitting.
   *
   * @param itBegin iterator to the beginning of the points (inclusive)
   * @param itEnd iterator to the end of the points (exclusive)
   * @param center reference to a variable to be filled with the normal of the line
   * @param radius reference to a variable to be filled with the distance of the line
   * @return average deviation of the points to the fitted line
   */
  float fitLineWithError(const std::vector<Vector2f>::const_iterator& itBegin, const std::vector<Vector2f>::const_iterator& itEnd, Vector2f& n0, float& d);

  /**
   * Fits a circle through the given points using least-squares fitting.
   *
   * @param points vector of points to be fitted to a circle
   * @param center reference to a variable to be filled with the center of the circle
   * @param radius reference to a variable to be filled with the radius of the circle
   */
  void fitCircle(const std::vector<Vector2f>& points, Vector2f& center, float& radius);
};
