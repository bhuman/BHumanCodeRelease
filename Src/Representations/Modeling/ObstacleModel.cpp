#include "ObstacleModel.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/RobotModel.h"
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

    ASSERT(std::isfinite(obstacle.velocity.x()));
    ASSERT(std::isfinite(obstacle.velocity.y()));

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
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:rectangle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:centerCross", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:leftRight", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:circle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:covariance", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:velocity", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:fallen", "drawingOnField");
  DEBUG_DRAWING3D("representation:ObstacleModel", "robot")
  {
    if(SystemCall::getMode() == SystemCall::remoteRobot
       && Blackboard::getInstance().exists("RawInertialSensorData")
       && Blackboard::getInstance().exists("RobotModel"))
    {
      const RawInertialSensorData& theRawInertialSensorData = static_cast<const RawInertialSensorData&>(Blackboard::getInstance()["RawInertialSensorData"]);
      const RobotModel& theRobotModel = static_cast<const RobotModel&>(Blackboard::getInstance()["RobotModel"]);
      const Vector3f orientation = theRawInertialSensorData.angle.cast<float>();
      TRANSLATE3D("representation:ObstacleModel", 0.f, 0.f, std::min(theRobotModel.soleLeft.translation.z(), theRobotModel.soleRight.translation.z()));
      ROTATE3D("representation:ObstacleModel", -orientation.x(), -orientation.y(), -orientation.z());
      RENDER_OPTIONS3D("representation:ObstacleModel", Drawings3D::disableOpacity | Drawings3D::disableDepth);
    }
  }

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
        color = ColorRGBA(200, 200, 200); // gray
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

    if(SystemCall::getMode() == SystemCall::remoteRobot)
      CYLINDER3D("representation:ObstacleModel", center.x(), center.y(), 290, 0, 0, 0, 100, 580, ColorRGBA(color.r, color.g, color.b, 128));
    else
      CYLINDER3D("representation:ObstacleModel", center.x(), center.y(), -210, 0, 0, 0, 0.5f * (left - right).norm(), 10, color);
    CROSS("representation:ObstacleModel:centerCross", center.x(), center.y(), Obstacle::getRobotDepth(), 10, Drawings::solidPen, color);

    float obstacleRadius = (left - right).norm() * .5f;
    Angle robotRotation = Blackboard::getInstance().exists("RobotPose") ? static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]).rotation : Angle();
    Vector2f frontRight(-Obstacle::getRobotDepth(), -obstacleRadius);
    frontRight = center + frontRight;
    RECTANGLE2("representation:ObstacleModel:rectangle", frontRight, obstacleRadius * 2, obstacleRadius * 2, -robotRotation, 16, Drawings::PenStyle::solidPen, ColorRGBA::black, Drawings::solidBrush, color);

    LINE("representation:ObstacleModel:leftRight", center.x(), center.y(), left.x(), left.y(), 20, Drawings::dottedPen, color);
    LINE("representation:ObstacleModel:leftRight", center.x(), center.y(), right.x(), right.y(), 20, Drawings::dottedPen, color);
    CIRCLE("representation:ObstacleModel:circle", center.x(), center.y(), obstacleRadius, 10, Drawings::dottedPen, color, Drawings::noBrush, color);
    COVARIANCE_ELLIPSES_2D("representation:ObstacleModel:covariance", obstacle.covariance, center);

    if(obstacle.velocity.squaredNorm() > 0)
      ARROW("representation:ObstacleModel:velocity", center.x(), center.y(),
            center.x() + 2 * obstacle.velocity.x(), center.y() + 2 * obstacle.velocity.y(), 10, Drawings::solidPen, ColorRGBA::black);

    if(obstacle.type >= Obstacle::fallenSomeRobot)
      DRAW_TEXT("representation:ObstacleModel:fallen", center.x(), center.y(), 100, color, "FALLEN");
  }

  DEBUG_DRAWING3D("representation:ObstacleModel:sectors", "robot")
  {
    if(Blackboard::getInstance().exists("RawInertialSensorData")
       && Blackboard::getInstance().exists("RobotModel"))
    {
      const RawInertialSensorData& theRawInertialSensorData = static_cast<const RawInertialSensorData&>(Blackboard::getInstance()["RawInertialSensorData"]);
      const RobotModel& theRobotModel = static_cast<const RobotModel&>(Blackboard::getInstance()["RobotModel"]);
      const Vector3f orientation = theRawInertialSensorData.angle.cast<float>();
      TRANSLATE3D("representation:ObstacleModel:sectors", 0.f, 0.f, std::min(theRobotModel.soleLeft.translation.z(), theRobotModel.soleRight.translation.z()));
      ROTATE3D("representation:ObstacleModel:sectors", -orientation.x(), -orientation.y(), -orientation.z());
      for(const Obstacle& obstacle : obstacles)
        if(!obstacle.isUnknown())
          RING_SECTOR3D("representation:ObstacleModel:sectors", Vector3f(0, 0, 3.f), obstacle.right.angle(), obstacle.left.angle(),
                        140.f, 160.f, obstacle.isTeammate() ? ownColor : opponentColor);
    }
  }
}
