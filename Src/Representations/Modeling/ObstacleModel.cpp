#include "ObstacleModel.h"
#include "Platform/SystemCall.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Math/Approx.h"
#include "Framework/Blackboard.h"

void ObstacleModel::verify() const
{
  DECLARE_DEBUG_RESPONSE("representation:ObstacleModel:verify");
  for(const auto& obstacle : obstacles)
  {
    ASSERT(std::isfinite(obstacle.center.x()));
    ASSERT(std::isfinite(obstacle.center.y()));

    ASSERT(std::isfinite(obstacle.left.x()));
    ASSERT(std::isfinite(obstacle.left.y()));

    ASSERT(std::isfinite(obstacle.right.x()));
    ASSERT(std::isfinite(obstacle.right.y()));

    DEBUG_RESPONSE("representation:ObstacleModel:verify")
      if((obstacle.left - obstacle.right).squaredNorm() < sqr(2000.f))
        OUTPUT_WARNING("Obstacle too big!");

    ASSERT(std::isnormal(obstacle.covariance(0, 0)));
    ASSERT(std::isnormal(obstacle.covariance(1, 1)));
    ASSERT(std::isfinite(obstacle.covariance(0, 1)));
    ASSERT(std::isfinite(obstacle.covariance(1, 0)));
    ASSERT(Approx::isEqual(obstacle.covariance(0, 1), obstacle.covariance(1, 0), 1e-20f));
  }
}

void ObstacleModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:covariance", "drawingOnField");

  // The ObstacleModel does not have a distinction between field players and goalkeepers, so they are all drawn in the field player color.
  const ColorRGBA ownColor = ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
      static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).ownTeam.fieldPlayerColor : GameState::Team::Color::black));

  const ColorRGBA opponentColor = ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
      static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).opponentTeam.fieldPlayerColor : GameState::Team::Color::red));

  ColorRGBA color;
  for(const auto& obstacle : obstacles)
  {
    switch(obstacle.type)
    {
      case Obstacle::fallenTeammate:
      case Obstacle::teammate:
      {
        color = ownColor;
        break;
      }
      case Obstacle::fallenOpponent:
      case Obstacle::opponent:
      {
        color = opponentColor;
        break;
      }
      case Obstacle::fallenSomeRobot:
      case Obstacle::someRobot:
      {
        color = ColorRGBA(134, 134, 134); // gray
        break;
      }
      default:
      {
        color = ColorRGBA::violet;
        break;
      }
    }
    const Vector2f& center = obstacle.center;
    const Vector2f& left = obstacle.left;
    const Vector2f& right = obstacle.right;

    CYLINDER3D("representation:ObstacleModel", center.x(), center.y(), -210, 0, 0, 0, 0.5f * (left - right).norm(), 10, color);

    const float robotDepth = Blackboard::getInstance().exists("RobotDimensions")
                             ? static_cast<const RobotDimensions&>(Blackboard::getInstance()["RobotDimensions"]).robotDepth : 150.f;
    Vector2f oPoints[4];
    oPoints[0] = obstacle.left.normalized(obstacle.left.norm() - robotDepth);
    oPoints[1] = obstacle.right.normalized(obstacle.right.norm() - robotDepth);
    oPoints[2] = obstacle.right.normalized(obstacle.right.norm() + robotDepth);
    oPoints[3] = obstacle.left.normalized(obstacle.left.norm() + robotDepth);
    POLYGON("representation:ObstacleModel", 4, oPoints, 0, Drawings::solidPen, color, Drawings::solidBrush, ColorRGBA(color.r/2, color.g/2, color.b/2));

    COVARIANCE_ELLIPSES_2D("representation:ObstacleModel:covariance", obstacle.covariance, center);
  }
}
