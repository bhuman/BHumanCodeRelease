/**
 * @file ObstacleScan.h
 *
 * This file declares a representation of a collection of perceived obstacle points per image column.
 *
 * @author Arne Hasselbring
 */

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include <vector>

STREAMABLE(ObstacleScan,
{
  void draw() const,

  (unsigned int)(0) xOffsetInImage, /**< The (image) x-coordinate of the first entry in the y-vector.*/
  (unsigned int)(0) xStepInImage, /**< The step of (image) x-coordinates between adjacent entries in the y-vector. */
  (std::vector<int>) yLowerInImage, /**< Vector of y-coordinates of obstacle points. -1 means that no obstacle has been found in that column. */

  (std::vector<Vector2f>) pointsOnField, /**< Obstacle points projected on the field. Only valid in columns in which yLowerInImage is not negative. */
});
