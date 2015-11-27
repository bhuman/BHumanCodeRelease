#include "ObstacleModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Random.h"
#include "Tools/Modeling/Obstacle.h"

void ObstacleModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:CenterCross", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:Circle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:Covariance", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:Velocity", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:Fallen", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:ObstacleModel", "robot");

  ColorRGBA color;

  for(const auto& obstacle : obstacles)
  {
    switch(obstacle.type)
    {
      case Obstacle::goalpost:
      {
        color = ColorRGBA::yellow;
        break;
      }
      case Obstacle::fallenTeammate:
      case Obstacle::teammate:
      {
        color = ColorRGBA::cyan;
        break;
      }
      case Obstacle::fallenOpponent:
      case Obstacle::opponent:
      {
        color = ColorRGBA::magenta;
        break;
      }
      case Obstacle::fallenSomeRobot:
      case Obstacle::someRobot:
      {
        color = ColorRGBA::orange;
        break;
      }
      default:
      {
        color = ColorRGBA::blue;
        break;
      }
    }
    CYLINDER3D("representation:ObstacleModel", obstacle.center.x(), obstacle.center.y(), -210, 0, 0, 0, (obstacle.left - obstacle.right).norm(), 10, color);
    CROSS("representation:ObstacleModel:CenterCross", obstacle.center.x(), obstacle.center.y(), Obstacle::getRobotDepth(), 10, Drawings::solidPen, color);
    CIRCLE("representation:ObstacleModel:Circle", obstacle.center.x(), obstacle.center.y(), (obstacle.left - obstacle.right).norm() / 2, 10, Drawings::dottedPen, color, Drawings::noBrush, color);
    COVARIANCE2D("representation:ObstacleModel:Covariance", (Matrix2f() << obstacle.covariance.topLeftCorner(2, 2)).finished(), obstacle.center);
    if(obstacle.velocity.squaredNorm() > 0)
      ARROW("representation:ObstacleModel:Velocity", obstacle.center.x(), obstacle.center.y(),
      obstacle.center.x() + 2 * obstacle.velocity.x(), obstacle.center.y() + 2 * obstacle.velocity.y(), 10, Drawings::solidPen, ColorRGBA::black);

    if(obstacle.type >= Obstacle::fallenSomeRobot)
    {
      DRAWTEXT("representation:ObstacleModel:Fallen", obstacle.center.x(), obstacle.center.y(), 100, color, "FALLEN");
    }
  }
}

ObstacleModelCompressed::ObstacleModelCompressed(const ObstacleModel& other, size_t maxNumberOfObstacles)
{
  const size_t numOfInputObstacles = other.obstacles.size();
  if(numOfInputObstacles <= maxNumberOfObstacles)
  {
    obstacles.reserve(numOfInputObstacles);
    for(const auto& obstacle : other.obstacles)
    {
      obstacles.emplace_back(obstacle);
    }
    return;
  }

  size_t offset = 0;
  size_t numOfUsedObstacles = numOfInputObstacles;
  if(numOfUsedObstacles > maxNumberOfObstacles)
  {
    numOfUsedObstacles = maxNumberOfObstacles;
    offset = static_cast<size_t>(random(static_cast<int>(numOfInputObstacles)));
  }
  obstacles.reserve(numOfUsedObstacles);
  for(size_t i = 0; i < numOfUsedObstacles; i++)
    obstacles.emplace_back(other.obstacles[(offset + i) % numOfInputObstacles]);
}

ObstacleModelCompressed::ObstacleCompressed::ObstacleCompressed(const Obstacle& other) : 
  covXX(other.covariance(0, 0)), covYY(other.covariance(1, 1)), covXY(other.covariance(0, 1)),
  center(other.center), left(other.left), right(other.right), type(other.type)
{}

void ObstacleModelCompressed::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ObstacleModelCompressed:CenterCross", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModelCompressed:Circle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModelCompressed:Fallen", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:ObstacleModelCompressed", "robot");

  ColorRGBA color;

  for(const auto& obstacle : obstacles)
  {
    switch(obstacle.type)
    {
      case Obstacle::goalpost:
      {
        color = ColorRGBA::yellow;
        break;
      }
      case Obstacle::fallenTeammate:
      case Obstacle::teammate:
      {
        color = ColorRGBA::cyan;
        break;
      }
      case Obstacle::fallenOpponent:
      case Obstacle::opponent:
      {
        color = ColorRGBA::magenta;
        break;
      }
      case Obstacle::fallenSomeRobot:
      case Obstacle::someRobot:
      {
        color = ColorRGBA::orange;
        break;
      }
      default:
      {
        color = ColorRGBA::blue;
        break;
      }
    }
    CYLINDER3D("representation:ObstacleModelCompressed", obstacle.center.x(), obstacle.center.y(), -210, 0, 0, 0, (obstacle.left - obstacle.right).norm(), 10, color);
    CROSS("representation:ObstacleModelCompressed:CenterCross", obstacle.center.x(), obstacle.center.y(), Obstacle::getRobotDepth(), 10, Drawings::solidPen, color);
    CIRCLE("representation:ObstacleModelCompressed:Circle", obstacle.center.x(), obstacle.center.y(), Obstacle::getRobotDepth(), 10, Drawings::dottedPen, color, Drawings::noBrush, color);

    if(obstacle.type >= Obstacle::fallenSomeRobot)
    {
      DRAWTEXT("representation:ObstacleModelCompressed:Fallen", obstacle.center.x(), obstacle.center.y(), 100, color, "FALLEN");
    }
  }
}
