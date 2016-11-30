#pragma once

#include "Tools/Math/Eigen.h"
#include <vector>

/**
* Fits a line through the given points using least-squares fitting.
*
* @param points vector of points to be fitted to a line
* @param center reference to a variable to be filled with the normal of the line
* @param radius reference to a variable to be filled with the distance of the line
* @return average absolute error of the fitted line to the line points
*/
float leastSquaresLineFit(const std::vector<Vector2f>& points, Vector2f& n0, float& d);

/**
* Fits a circle through the given points using least-squares fitting.
*
* @param points vector of points to be fitted to a circle
* @param center reference to a variable to be filled with the center of the circle
* @param radius reference to a variable to be filled with the radius of the circle
* @return normed (0..1) average error of the fitted circle to the circle points
*/
float leastSquaresCircleFit(const std::vector<Vector2f>& points, Vector2f& center, float& radius);
