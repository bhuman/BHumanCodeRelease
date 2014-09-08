#include "ExpObstacleModel.h"
#include "Tools/Debugging/DebugDrawings.h"

#define feetRadius 110.f
#define shoulderRadius 140.f
#define goalPostRadius 50.f

void ExpObstacleModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ExpObstacleModel:center", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ExpObstacleModel:feetCircle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ExpObstacleModel:shoulderCircle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ExpObstacleModel:covariance", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ExpObstacleModel:velocity", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ExpObstacleModel:fallen", "drawingOnField");

  ColorRGBA color;

  for(const auto& obstacle : eobs)
  {
    switch(obstacle.type)
    {
    case ExpObstacle::GOALPOST:
    {
      color = ColorRGBA::yellow;
      CROSS("representation:ExpObstacleModel:center", obstacle.center.x, obstacle.center.y, 50, 10, Drawings::ps_solid, ColorRGBA::yellow);
      COVARIANCE2D("representation:ExpObstacleModel:covariance",
                   Matrix2x2<>(obstacle.covariance[0][0], obstacle.covariance[1][0], obstacle.covariance[0][1], obstacle.covariance[1][1]), obstacle.center);
      continue;
    }
    case ExpObstacle::FALLENBLUE:
    case ExpObstacle::ROBOTBLUE:
    {
      color = ColorRGBA::blue;
      break;
    }
    case ExpObstacle::FALLENRED:
    case ExpObstacle::ROBOTRED:
    {
      color = ColorRGBA::red;
      break;
    }
    case ExpObstacle::SOMEFALLENROBOT:
    case ExpObstacle::SOMEROBOT:
    {
      color = ColorRGBA::white;
      break;
    }
    default:
    {
      color = ColorRGBA::black;
      break;
    }
    }
    CROSS("representation:ExpObstacleModel:center", obstacle.center.x, obstacle.center.y, 20, 10, Drawings::ps_solid, color);
    CIRCLE("representation:ExpObstacleModel:feetCircle", obstacle.center.x, obstacle.center.y, feetRadius, 10,
           Drawings::ps_solid, color, Drawings::bs_solid, color);
    CIRCLE("representation:ExpObstacleModel:shoulderCircle", obstacle.center.x, obstacle.center.y, shoulderRadius, 10,
           Drawings::ps_solid, color, Drawings::bs_solid, color);
    COVARIANCE2D("representation:ExpObstacleModel:covariance",
                 Matrix2x2<>(obstacle.covariance[0][0], obstacle.covariance[1][0], obstacle.covariance[0][1], obstacle.covariance[1][1]), obstacle.center);
    if(obstacle.velocity.abs() > 0)
      ARROW("representation:ExpObstacleModel:velocity", obstacle.center.x, obstacle.center.y, obstacle.center.x + (obstacle.velocity.x * 1000.f), obstacle.center.y + (obstacle.velocity.y * 1000.f), 10, Drawings::ps_solid, color);

    if(obstacle.type == ExpObstacle::SOMEFALLENROBOT || obstacle.type == ExpObstacle::FALLENBLUE || obstacle.type == ExpObstacle::FALLENRED)
    {
      DRAWTEXT("representation:ExpObstacleModel:fallen", obstacle.center.x, obstacle.center.y, 100, color, "FALLEN");
    }
  }
}

Vector2<> ExpObstacleModel::ExpObstacle::getLeftFoot() const
{
  Vector2<> result = center;
  result.normalize(type == GOALPOST ? goalPostRadius : feetRadius);
  result.rotateLeft();
  return center + result;
}

Vector2<> ExpObstacleModel::ExpObstacle::getRightFoot() const
{
  Vector2<> result = center;
  result.normalize(type == GOALPOST ? goalPostRadius : feetRadius);
  result.rotateRight();
  return center + result;
}

Vector2<> ExpObstacleModel::ExpObstacle::getLeftShoulder() const
{
  Vector2<> result = center;
  result.normalize(type == GOALPOST ? goalPostRadius : shoulderRadius);
  result.rotateLeft();
  return center + result;
}

Vector2<> ExpObstacleModel::ExpObstacle::getRightShoulder() const
{
  Vector2<> result = center;
  result.normalize(type == GOALPOST ? goalPostRadius : shoulderRadius);
  result.rotateRight();
  return center + result;
}
