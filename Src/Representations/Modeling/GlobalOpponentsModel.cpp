/**
* @file GlobalOpponentsModel.cpp
*
* Implementation of a representation that stores the estimated states
* of the opponent robots.
*
* @author Tim Laue
* @author Michelle Gusev
*/

#include "GlobalOpponentsModel.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Framework/Blackboard.h"
#include "Math/Approx.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Modeling/Obstacle.h"

void GlobalOpponentsModel::verify() const
{
  for([[maybe_unused]] const auto& opponent : opponents)
  {
    ASSERT(std::isfinite(opponent.position.x()));
    ASSERT(std::isfinite(opponent.position.y()));
    ASSERT(std::isfinite(opponent.left.x()));
    ASSERT(std::isfinite(opponent.left.y()));
    ASSERT(std::isfinite(opponent.right.x()));
    ASSERT(std::isfinite(opponent.right.y()));
  }
}

void GlobalOpponentsModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:GlobalOpponentsModel", "drawingOnField");
  if(SystemCall::getMode() != SystemCall::remoteRobot)
    DECLARE_DEBUG_DRAWING3D("representation:GlobalOpponentsModel", "field");

  const ColorRGBA opponentColor = ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
    static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).opponentTeam.fieldPlayerColor : GameState::Team::Color::red));

  for(const auto& opponent : opponents)
  {
    const Vector2f& center = opponent.position;
    const Vector2f& left = opponent.left;
    const Vector2f& right = opponent.right;
    const ColorRGBA fillColor = opponent.unidentified ? ColorRGBA::white : opponentColor;

    if(SystemCall::getMode() != SystemCall::remoteRobot)
      CYLINDER3D("representation:GlobalOpponentsModel", center.x(), center.y(), -210, 0, 0, 0, (left - right).norm(), 10, fillColor);

    float obstacleRadius = (left - right).norm() * .5f;
    Vector2f frontRight(-Obstacle::getRobotDepth(), -obstacleRadius);
    frontRight = center + frontRight;
    RECTANGLE2("representation:GlobalOpponentsModel", frontRight, obstacleRadius * 2, obstacleRadius * 2, 0.f, 16, Drawings::PenStyle::solidPen, ColorRGBA::black, Drawings::solidBrush, fillColor);
    if(opponent.unidentified)
      DRAW_TEXT("representation:GlobalOpponentsModel", center.x()-20, center.y()-50, 150, ColorRGBA::black, "?");
  }

  if(SystemCall::getMode() == SystemCall::remoteRobot)
    DEBUG_DRAWING3D("representation:GlobalOpponentsModel", "robot")
    {
      if(Blackboard::getInstance().exists("GameState")
         && Blackboard::getInstance().exists("RawInertialSensorData")
         && Blackboard::getInstance().exists("RobotModel")
         && Blackboard::getInstance().exists("RobotPose"))
      {
        const GameState theGameState = static_cast<const GameState&>(Blackboard::getInstance()["GameState"]);
        const RawInertialSensorData& theRawInertialSensorData = static_cast<const RawInertialSensorData&>(Blackboard::getInstance()["RawInertialSensorData"]);
        const RobotModel& theRobotModel = static_cast<const RobotModel&>(Blackboard::getInstance()["RobotModel"]);
        const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
        const Vector3f orientation = theRawInertialSensorData.angle.cast<float>();
        TRANSLATE3D("representation:GlobalOpponentsModel", 0.f, 0.f, std::min(theRobotModel.soleLeft.translation.z(), theRobotModel.soleRight.translation.z()));
        ROTATE3D("representation:GlobalOpponentsModel", -orientation.x(), -orientation.y(), -orientation.z());
        RENDER_OPTIONS3D("representation:GlobalOpponentsModel", Drawings3D::disableOpacity | Drawings3D::disableDepth);
        ColorRGBA color = ColorRGBA::fromTeamColor(static_cast<int>(theGameState.opponentTeam.fieldPlayerColor));
        color.a = 128;
        ColorRGBA white = ColorRGBA::white;
        white.a = 128;
        for(const OpponentEstimate& opponent : opponents)
        {
          const Vector2f position = theRobotPose.inverse() * opponent.position;
          CYLINDER3D("representation:GlobalOpponentsModel", position.x(), position.y(), 290, 0, 0, 0, 100, 580, opponent.unidentified ? white : color);
        }
      }
    }

  DEBUG_DRAWING3D("representation:GlobalOpponentsModel:sectors", "robot")
  {
    if(Blackboard::getInstance().exists("GameState")
       && Blackboard::getInstance().exists("RawInertialSensorData")
       && Blackboard::getInstance().exists("RobotModel")
       && Blackboard::getInstance().exists("RobotPose"))
    {
      const GameState theGameState = static_cast<const GameState&>(Blackboard::getInstance()["GameState"]);
      const RawInertialSensorData& theRawInertialSensorData = static_cast<const RawInertialSensorData&>(Blackboard::getInstance()["RawInertialSensorData"]);
      const RobotModel& theRobotModel = static_cast<const RobotModel&>(Blackboard::getInstance()["RobotModel"]);
      const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
      const Vector3f orientation = theRawInertialSensorData.angle.cast<float>();
      TRANSLATE3D("representation:GlobalOpponentsModel:sectors", 0.f, 0.f, std::min(theRobotModel.soleLeft.translation.z(), theRobotModel.soleRight.translation.z()));
      ROTATE3D("representation:GlobalOpponentsModel:sectors", -orientation.x(), -orientation.y(), -orientation.z());
      const ColorRGBA color = ColorRGBA::fromTeamColor(static_cast<int>(theGameState.opponentTeam.fieldPlayerColor));
      for(const OpponentEstimate& opponent : opponents)
      {
        const Vector2f position = theRobotPose.inverse() * opponent.position;
        const Angle range = std::max(1_deg, static_cast<Angle>(std::asin(std::min(1.f, 100.f / position.norm()))));
        const Angle angle = position.angle();
        RING_SECTOR3D("representation:GlobalOpponentsModel:sectors", Vector3f(0, 0, 3.f), angle - range, angle + range, 140.f, 160.f, opponent.unidentified ? ColorRGBA::white : color);
      }
    }
  }
}
