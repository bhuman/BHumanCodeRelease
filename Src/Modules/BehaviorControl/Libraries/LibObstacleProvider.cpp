/**
 * @file LibObstacleProvider.cpp
 * @author Andreas Stolpmann
 */

#include "LibObstacleProvider.h"
#include "Math/Geometry.h"
#include "Math/BHMath.h"

MAKE_MODULE(LibObstacleProvider, behaviorControl);

void LibObstacleProvider::update(LibObstacle& libObstacle)
{
  libObstacle.isObstacleInPath = [this](const Vector2f& target, const float pathWidth, const float maxDistance) -> bool
  {
    return isObstacleInPath(target, pathWidth, maxDistance);
  };

  libObstacle.isObstacleOnSide = [this](const bool left) -> bool
  {
    return isObstacleOnSide(left);
  };
}

bool LibObstacleProvider::isObstacleOnSide(const bool left) const
{
  const Vector2f bottomLeftCorner(xMin, left ? yMax : -yMax);
  const Vector2f topRightCorner(xMax, left ? yMin : -yMin);

  for(const auto& obstacle : theObstacleModel.obstacles)
  {
    if(Geometry::isPointInsideRectangle2(bottomLeftCorner, topRightCorner, obstacle.center)
       || Geometry::isPointInsideRectangle2(bottomLeftCorner, topRightCorner, obstacle.left)
       || Geometry::isPointInsideRectangle2(bottomLeftCorner, topRightCorner, obstacle.right))
      return true;
  }
  return false;
}

bool LibObstacleProvider::isObstacleInPath(const Vector2f& target, const float pathWidth, const float maxDistance) const
{
  const float angle = target.angle();

  Vector2f start(-100.f, 0.f);
  start.rotate(angle);
  Vector2f end(target);
  end += Vector2f(100.f, 0.f).rotate(angle);

  const float distanceSqr = end.squaredNorm();

  Vector2f points[4];
  points[0] = start + Vector2f(0.f, pathWidth / 2).rotate(angle);
  points[1] = start + Vector2f(0.f, -pathWidth / 2).rotate(angle);
  points[2] = end + Vector2f(0.f, -pathWidth / 2).rotate(angle);
  points[3] = end + Vector2f(0.f, pathWidth / 2).rotate(angle);

  const float maxDistanceSqr = distanceSqr < sqr(maxDistance) ? distanceSqr : sqr(maxDistance);
  for(const Obstacle& obstacle : theObstacleModel.obstacles)
  {
    if(obstacle.center.squaredNorm() < maxDistanceSqr &&
       (Geometry::isPointInsideConvexPolygon(points, 4, obstacle.center) ||
        Geometry::isPointInsideConvexPolygon(points, 4, obstacle.left) ||
        Geometry::isPointInsideConvexPolygon(points, 4, obstacle.right)))
      return true;
  }
  return false;
}
