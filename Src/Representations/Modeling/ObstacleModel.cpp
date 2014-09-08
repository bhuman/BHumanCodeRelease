/**
* @file ObstacleModel.cpp
* Implementation of class ObstacleModel
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "ObstacleModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/BHMath.h"

ObstacleModel::Obstacle::Obstacle(const Vector2<>& leftCorner, const Vector2<>& rightCorner,
                                  const Vector2<>& center, const Vector2<>& closestPoint,
                                  const Matrix2x2<>& covariance, Type type)
: leftCorner(leftCorner),
  rightCorner(rightCorner),
  center(center),
  closestPoint(closestPoint),
  covariance(covariance), type(type) {}

void ObstacleModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:Center", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModelReduced", "drawingOnField");

  COMPLEX_DRAWING("representation:ObstacleModel",
  {
    for(const auto& obstacle : obstacles)
    {
      const ColorRGBA color = obstacle.type == Obstacle::US ? ColorRGBA::blue : obstacle.type == Obstacle::ARM ? ColorRGBA::yellow : obstacle.type == Obstacle::FOOT ? ColorRGBA::green : ColorRGBA::black;
      const Vector2<>& left = obstacle.leftCorner;
      Vector2<> right = obstacle.rightCorner;
      const float leftLen = left.abs();
      const float rightLen = right.abs();

      Vector2<> expanded = left;
      expanded.normalize(leftLen + 500.f);
      LINE("representation:ObstacleModel", left.x, left.y, expanded.x, expanded.y, 30, Drawings::ps_solid, color);
      expanded = right;
      expanded.normalize(rightLen + 500.f);
      LINE("representation:ObstacleModel", right.x, right.y, expanded.x, expanded.y, 30, Drawings::ps_solid, color);

      Vector2<> rightNorm = right;
      rightNorm.normalize();
      Vector2<> leftRotated = Vector2<>(rightNorm.x, -rightNorm.y) * left.x + Vector2<>(rightNorm.y, rightNorm.x) * left.y;
      float angle = std::atan2(leftRotated.y, leftRotated.x);
      if(angle < 0.f)
        angle += pi2;

      int segments = ((int) floor(angle / fromDegrees(30.f))) + 1;
      float segmentAngle = angle / (float) segments;
      Vector2<> newRight;
      for(int i = 1; i < segments; ++i)
      {
        newRight = right;
        newRight.rotate(segmentAngle);
        newRight.normalize(rightLen + (leftLen - rightLen) * i / (float) segments);
        LINE("representation:ObstacleModel", right.x, right.y, newRight.x, newRight.y, 30, Drawings::ps_solid, color);
        right = newRight;
      }
      LINE("representation:ObstacleModel", left.x, left.y, right.x, right.y, 30, Drawings::ps_solid, color);

      CROSS("representation:ObstacleModel", obstacle.center.x, obstacle.center.y, 100, 20, Drawings::ps_solid, ColorRGBA::blue);
      CROSS("representation:ObstacleModel", obstacle.closestPoint.x, obstacle.closestPoint.y, 100, 20, Drawings::ps_solid, ColorRGBA::red);
    }
  });

  //draws the center of the obstacle model on a 2d scene
  COMPLEX_DRAWING("representation:ObstacleModel:Center",
  {
    ColorRGBA blue = ColorRGBA::blue;
    blue.a = 75;
    ColorRGBA red = ColorRGBA::red;
    red.a = 75;
    for(const auto& obstacle : obstacles)
    {
      CROSS("representation:ObstacleModel:Center", obstacle.center.x, obstacle.center.y, 100, 20, Drawings::ps_solid, blue);
      CROSS("representation:ObstacleModel:Center", obstacle.closestPoint.x, obstacle.closestPoint.y, 100, 20, Drawings::ps_solid, red);
    }
  });
  //draws the obstacle model on a 2d scene
  COMPLEX_DRAWING("representation:ObstacleModelReduced",
  {
    for(const auto& obstacle : obstacles)
    {
      const ColorRGBA color = obstacle.type == Obstacle::US ? ColorRGBA::blue : obstacle.type == Obstacle::ARM ? ColorRGBA::yellow : ColorRGBA::black;
      const Vector2<>& left = obstacle.leftCorner;
      Vector2<> right = obstacle.rightCorner;
      const float leftLen = left.abs();
      const float rightLen = right.abs();

      Vector2<> expanded = left;
      expanded.normalize(leftLen + 500.f);
      LINE("representation:ObstacleModelReduced", left.x, left.y, expanded.x, expanded.y, 30, Drawings::ps_solid, color);
      expanded = right;
      expanded.normalize(rightLen + 500.f);
      LINE("representation:ObstacleModelReduced", right.x, right.y, expanded.x, expanded.y, 30, Drawings::ps_solid, color);

      Vector2<> rightNorm = right;
      rightNorm.normalize();
      Vector2<> leftRotated = Vector2<>(rightNorm.x, -rightNorm.y) * left.x + Vector2<>(rightNorm.y, rightNorm.x) * left.y;
      float angle = std::atan2(leftRotated.y, leftRotated.x);
      if(angle < 0.f)
        angle += pi2;

      int segments = ((int) floor(angle / fromDegrees(30.f))) + 1;
      float segmentAngle = angle / (float) segments;
      Vector2<> newRight;
      for(int i = 1; i < segments; ++i)
      {
        newRight = right;
        newRight.rotate(segmentAngle);
        newRight.normalize(rightLen + (leftLen - rightLen) * i / (float) segments);
        LINE("representation:ObstacleModelReduced", right.x, right.y, newRight.x, newRight.y, 30, Drawings::ps_solid, color);
        right = newRight;
      }
      LINE("representation:ObstacleModelReduced", left.x, left.y, right.x, right.y, 30, Drawings::ps_solid, color);
    }
  });
  //draws the obstacle model on a 3d scene
  DECLARE_DEBUG_DRAWING3D("representation:ObstacleModel", "robot",
  {
    TRANSLATE3D("representation:ObstacleModel", 0, 0, -200);
    for(const auto& obstacle : obstacles)
    {
      const ColorRGBA color = obstacle.type == Obstacle::US ? ColorRGBA::blue : obstacle.type == Obstacle::ARM ? ColorRGBA::yellow : ColorRGBA::black;
      const Vector2<>& left = obstacle.leftCorner;
      Vector2<> right = obstacle.rightCorner;
      const float leftLen = left.abs();
      const float rightLen = right.abs();

      Vector2<> expanded = left;
      expanded.normalize(leftLen + 500.f);
      LINE3D("representation:ObstacleModel", left.x, left.y, 0, expanded.x, expanded.y, 0, 4, color);
      expanded = right;
      expanded.normalize(rightLen + 500.f);
      LINE3D("representation:ObstacleModel", right.x, right.y, 0, expanded.x, expanded.y, 0, 4, color);

      Vector2<> rightNorm = right;
      rightNorm.normalize();
      Vector2<> leftRotated = Vector2<>(rightNorm.x, -rightNorm.y) * left.x + Vector2<>(rightNorm.y, rightNorm.x) * left.y;
      float angle = std::atan2(leftRotated.y, leftRotated.x);
      if(angle < 0.f)
        angle += pi2;

      int segments = ((int) floor(angle / fromDegrees(30.f))) + 1;
      float segmentAngle = angle / (float) segments;
      Vector2<> newRight;
      for(int i = 1; i < segments; ++i)
      {
        newRight = right;
        newRight.rotate(segmentAngle);
        newRight.normalize(rightLen + (leftLen - rightLen) * i / (float) segments);
        LINE3D("representation:ObstacleModel", right.x, right.y, 0, newRight.x, newRight.y, 0, 4, color);
        right = newRight;
      }
      LINE3D("representation:ObstacleModel", left.x, left.y, 0, right.x, right.y, 0, 4, color);
    }
  });

  //draws the center of the obstacle model on a 3d scene
  DECLARE_DEBUG_DRAWING3D("representation:ObstacleModel:Center", "robot",
  {
    TRANSLATE3D("representation:ObstacleModel:Center", 0, 0, -230);
    for(const auto& obstacle : obstacles)
    {
      ColorRGBA blue = ColorRGBA::blue;
      blue.a = 75;
      ColorRGBA red = ColorRGBA::red;
      red.a = 75;
      CROSS3D("representation:ObstacleModel:Center", obstacle.center.x, obstacle.center.y, 1.f, 50, 2.f, blue);
      CROSS3D("representation:ObstacleModel:Center", obstacle.closestPoint.x, obstacle.closestPoint.y, 1.f, 50, 2.f, red);
      LINE3D("representation:ObstacleModel:Center", 0, 0, 1.f, obstacle.closestPoint.x, obstacle.closestPoint.y, 1.f, 2.f, blue);
    }
  });
}

ObstacleModelCompressed::Obstacle::Obstacle(const ObstacleModel::Obstacle& other)
: leftCorner(other.leftCorner),
  rightCorner(other.rightCorner),
  center(other.center),
  closestPoint(other.closestPoint),
  x11(other.covariance.c[0].x),
  x12(other.covariance.c[1].x),x22(other.covariance.c[1].y),
  type(other.type) {}

ObstacleModelCompressed::ObstacleModelCompressed(const ObstacleModel& other, unsigned int maxNumberOfObstacles)
{
  unsigned int offset = 0;
  const size_t numOfInputObstacles = other.obstacles.size();
  size_t numOfUsedObstacles = numOfInputObstacles;
  if(numOfUsedObstacles > maxNumberOfObstacles)
  {
    numOfUsedObstacles = maxNumberOfObstacles;
    offset = static_cast<unsigned int>(random(static_cast<int>(numOfInputObstacles)));
  }
  obstacles.reserve(numOfUsedObstacles);
  for(size_t i = 0; i < numOfUsedObstacles; i++)
    obstacles.push_back(Obstacle(other.obstacles[(offset + i) % numOfInputObstacles]));
}

ObstacleModelCompressed::operator ObstacleModel() const
{
  ObstacleModel o;
  for(const auto& compressed : obstacles)
  {
    //x12 and x21 are identical, therefore only x12 is streamed.
    Matrix2x2<> uncompressedCovariance(compressed.x11, compressed.x12,
                                       compressed.x12, compressed.x22);

    ObstacleModel::Obstacle uncompressed(Vector2<>(compressed.leftCorner),
                                         Vector2<>(compressed.rightCorner),
                                         Vector2<>(compressed.center),
                                         Vector2<>(compressed.closestPoint),
                                         uncompressedCovariance,
                                         compressed.type);
    o.obstacles.push_back(uncompressed);
  }
  return o;
}
