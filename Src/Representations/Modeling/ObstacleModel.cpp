/**
* @file ObstacleModel.cpp
* Implementation of class ObstacleModel
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "ObstacleModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Pose3D.h"

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
  DECLARE_DEBUG_DRAWING("representation:ObstacleModelReduced", "drawingOnField");

  COMPLEX_DRAWING("representation:ObstacleModel",
  {
    for(std::vector<Obstacle>::const_iterator it = obstacles.begin(), end = obstacles.end(); it != end; ++it)
    {
      const Obstacle& obstacle = *it;
      ColorClasses::Color color = obstacle.type == Obstacle::US ? ColorClasses::blue : obstacle.type == Obstacle::ARM ? ColorClasses::yellow : obstacle.type == Obstacle::FOOT ? ColorClasses::green : ColorClasses::black;
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

      CROSS("representation:ObstacleModel", obstacle.center.x, obstacle.center.y, 100, 20, Drawings::ps_solid, ColorClasses::blue);
      CROSS("representation:ObstacleModel", obstacle.closestPoint.x, obstacle.closestPoint.y, 100, 20, Drawings::ps_solid, ColorClasses::red);
    }
  });

  COMPLEX_DRAWING("representation:ObstacleModelReduced",
  {
    for(std::vector<Obstacle>::const_iterator it = obstacles.begin(), end = obstacles.end(); it != end; ++it)
    {
      const Obstacle& obstacle = *it;
      ColorClasses::Color color = obstacle.type == Obstacle::US ? ColorClasses::blue : obstacle.type == Obstacle::ARM ? ColorClasses::yellow : ColorClasses::black;
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
}

void ObstacleModel::draw3D(const Pose3D& torsoMatrix) const
{

#define LINE3D_REL_ODOMETRY_ORIGIN(id, torsoMatrix, point1, point2, size, color) \
  { \
    Pose3D torsoMatrixInv = torsoMatrix.invert(); \
    Vector3<> point1Rel = torsoMatrixInv * Vector3<>(point1.x, point1.y, 0.f); \
    Vector3<> point2Rel = torsoMatrixInv * Vector3<>(point2.x, point2.y, 0.f); \
    LINE3D(id, point1Rel.x, point1Rel.y, 0.f, point2Rel.x, point2Rel.y, 0.f, size, color); \
  }

  DECLARE_DEBUG_DRAWING3D("representation:ObstacleModel", "robot",
    for(std::vector<Obstacle>::const_iterator it = obstacles.begin(), end = obstacles.end(); it != end; ++it)
    {
      const Obstacle& obstacle = *it;
      ColorClasses::Color color = obstacle.type == Obstacle::US ? ColorClasses::blue : obstacle.type == Obstacle::ARM ? ColorClasses::yellow : ColorClasses::black;
      const Vector2<>& left = obstacle.leftCorner;
      Vector2<> right = obstacle.rightCorner;
      const float leftLen = left.abs();
      const float rightLen = right.abs();

      Vector2<> expanded = left;
      expanded.normalize(leftLen + 500.f);
      LINE3D_REL_ODOMETRY_ORIGIN("representation:ObstacleModel", torsoMatrix, left, expanded, 4, color);
      expanded = right;
      expanded.normalize(rightLen + 500.f);
      LINE3D_REL_ODOMETRY_ORIGIN("representation:ObstacleModel", torsoMatrix, right, expanded, 4, color);

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
        LINE3D_REL_ODOMETRY_ORIGIN("representation:ObstacleModel", torsoMatrix, right, newRight, 4, color);
        right = newRight;
      }
      LINE3D_REL_ODOMETRY_ORIGIN("representation:ObstacleModel", torsoMatrix, left, right, 4, color);
    }
  );
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
  const unsigned int numOfInputObstacles = other.obstacles.size();
  unsigned int numOfUsedObstacles = numOfInputObstacles;
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
  for(std::vector<Obstacle>::const_iterator it = obstacles.begin(); it != obstacles.end(); it++)
  {
    const Obstacle& compressed = *it;

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


