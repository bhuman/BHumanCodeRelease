/**
* @file GlobalTeammatesModel.cpp
*
* Implementation of a representation that stores the estimated states
* of the teammates.
* The representation is a result of the combination of receveived position information
* and own observations of teammates.
*
* @author Tim Laue
*/

#include "Framework/Blackboard.h"
#include "GlobalTeammatesModel.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/RobotModel.h"

Vector2f GlobalTeammatesModel::TeammateEstimate::getFuturePosition(unsigned time) const
{
  return Teammate::getEstimatedPosition(pose, relativeWalkTarget, speed, time);
};

void GlobalTeammatesModel::verify() const
{
  for(std::size_t i=0; i < teammates.size(); i++)
  {
    const TeammateEstimate& tm = teammates.at(i);
    [[maybe_unused]] const Pose2f& p = tm.pose;
    ASSERT(std::isfinite(p.translation.x()));
    ASSERT(std::isfinite(p.translation.y()));
    ASSERT(std::isfinite(p.rotation));
    ASSERT(p.rotation >= -pi);
    ASSERT(p.rotation <= pi);
    ASSERT(std::isfinite(tm.relativeWalkTarget.x()));
    ASSERT(std::isfinite(tm.relativeWalkTarget.y()));
    ASSERT(std::isfinite(tm.speed));
    ASSERT(tm.speed >= 0);
    if(i > 0)
    {
      [[maybe_unused]] const TeammateEstimate& tmPrev = teammates.at(i-1);
      ASSERT(tm.playerNumber > tmPrev.playerNumber);
    }
  }
}

void GlobalTeammatesModel::draw() const
{
  DEBUG_DRAWING("representation:GlobalTeammatesModel", "drawingOnField")
  {
    const ColorRGBA ownFieldPlayerColorForDrawing = ColorRGBA::fromTeamColor(
      static_cast<int>(Blackboard::getInstance().exists("GameState") ? static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).ownTeam.fieldPlayerColor : GameState::Team::Color::black));
    ColorRGBA ownFieldPlayerColorForCircleDrawing = ownFieldPlayerColorForDrawing;
    ownFieldPlayerColorForCircleDrawing.a = 100;

    const ColorRGBA ownGoalkeeperColorForDrawing = ColorRGBA::fromTeamColor(
      static_cast<int>(Blackboard::getInstance().exists("GameState") ? static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).ownTeam.goalkeeperColor : GameState::Team::Color::black));
    ColorRGBA ownGoalkeeperColorForCircleDrawing = ownGoalkeeperColorForDrawing;
    ownGoalkeeperColorForCircleDrawing.a = 100;

    const int goalkeeperNumber = Blackboard::getInstance().exists("GameState") ? static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).ownTeam.goalkeeperNumber : -1;

    for(const TeammateEstimate& t : teammates)
    {
      const Vector2f dirVec = t.pose * Vector2f(200, 0);
      if(t.playerNumber == goalkeeperNumber)
      {
        CIRCLE("representation:GlobalTeammatesModel", t.pose.translation.x(), t.pose.translation.y(), 300, 1, Drawings::solidPen, ownGoalkeeperColorForCircleDrawing, Drawings::solidBrush, ownGoalkeeperColorForCircleDrawing);
        ROBOT("representation:GlobalTeammatesModel", t.pose, dirVec, dirVec, 1.f, ownGoalkeeperColorForDrawing,
              ColorRGBA(255, 255, 255, 128), ColorRGBA(0, 0, 0, 0));
      }
      else
      {
        CIRCLE("representation:GlobalTeammatesModel", t.pose.translation.x(), t.pose.translation.y(), 300, 1, Drawings::solidPen, ownFieldPlayerColorForCircleDrawing, Drawings::solidBrush, ownFieldPlayerColorForCircleDrawing);
        ROBOT("representation:GlobalTeammatesModel", t.pose, dirVec, dirVec, 1.f, ownFieldPlayerColorForDrawing,
              ColorRGBA(255, 255, 255, 128), ColorRGBA(0, 0, 0, 0));
      }
      DRAW_TEXT("representation:GlobalTeammatesModel", t.pose.translation.x() + 200, t.pose.translation.y(),
            250, ColorRGBA::white, t.playerNumber);
    }
  }

  DEBUG_DRAWING("representation:GlobalTeammatesModel:walkTargets", "drawingOnField")
  {
    for(const TeammateEstimate& t : teammates)
    {
      const Vector2f walkTarget = t.pose * t.relativeWalkTarget;
      LINE("representation:GlobalTeammatesModel:walkTargets", t.pose.translation.x(), t.pose.translation.y(), walkTarget.x(), walkTarget.y(),
           10, Drawings::dashedPen, ColorRGBA::magenta);
    }
  }

  DEBUG_DRAWING3D("representation:GlobalTeammatesModel", "robot")
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
      TRANSLATE3D("representation:GlobalTeammatesModel", 0.f, 0.f, std::min(theRobotModel.soleLeft.translation.z(), theRobotModel.soleRight.translation.z()));
      ROTATE3D("representation:GlobalTeammatesModel", -orientation.x(), -orientation.y(), -orientation.z());
      RENDER_OPTIONS3D("representation:GlobalTeammatesModel", Drawings3D::disableOpacity | Drawings3D::disableDepth);
      ColorRGBA fieldPlayerColor = ColorRGBA::fromTeamColor(static_cast<int>(theGameState.ownTeam.fieldPlayerColor));
      ColorRGBA goalkeeperColor = ColorRGBA::fromTeamColor(static_cast<int>(theGameState.ownTeam.goalkeeperColor));
      fieldPlayerColor.a = goalkeeperColor.a = 128;
      for(const TeammateEstimate& teammate : teammates)
      {
        const ColorRGBA color = teammate.playerNumber == theGameState.ownTeam.goalkeeperNumber ? goalkeeperColor : fieldPlayerColor;
        ColorRGBA targetColor = color;
        targetColor.a = 224;

        const Pose2f walkTarget = theRobotPose.inverse() * teammate.pose * teammate.relativeWalkTarget;
        const Vector2f points[] =
        {
          walkTarget * Vector2f(100.f, -20.f),
          walkTarget * Vector2f(100.f, 20.f),
          walkTarget * Vector2f(-100.f, 20.f),
          walkTarget * Vector2f(-100.f, -20.f),
          walkTarget * Vector2f(20.f, -100.f),
          walkTarget * Vector2f(20.f, -20.f),
          walkTarget * Vector2f(-20.f, -20.f),
          walkTarget * Vector2f(-20.f, -100.f),
          walkTarget * Vector2f(20.f, 20.f),
          walkTarget * Vector2f(20.f, 100.f),
          walkTarget * Vector2f(-20.f, 100.f),
          walkTarget * Vector2f(-20.f, 20.f)
        };

        for(int i = 0; i < 12; i += 4)
          QUAD3D("representation:GlobalTeammatesModel",
                 Vector3f(points[i].x(), points[i].y(), 2.f),
                 Vector3f(points[i + 1].x(), points[i + 1].y(), 2.f),
                 Vector3f(points[i + 2].x(), points[i + 2].y(), 2.f),
                 Vector3f(points[i + 3].x(), points[i + 3].y(), 2.f),
                 targetColor);
        const Vector2f position = theRobotPose.inverse() * teammate.pose.translation;
        CYLINDER3D("representation:GlobalTeammatesModel", position.x(), position.y(), 290, 0, 0, 0, 100, 580, color);

      }
    }
  }

  DEBUG_DRAWING3D("representation:GlobalTeammatesModel:sectors", "robot")
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
      TRANSLATE3D("representation:GlobalTeammatesModel:sectors", 0.f, 0.f, std::min(theRobotModel.soleLeft.translation.z(), theRobotModel.soleRight.translation.z()));
      ROTATE3D("representation:GlobalTeammatesModel:sectors", -orientation.x(), -orientation.y(), -orientation.z());
      const ColorRGBA fieldPlayerColor = ColorRGBA::fromTeamColor(static_cast<int>(theGameState.ownTeam.fieldPlayerColor));
      const ColorRGBA goalkeeperColor = ColorRGBA::fromTeamColor(static_cast<int>(theGameState.ownTeam.goalkeeperColor));
      for(const TeammateEstimate& teammate : teammates)
      {
        const Vector2f position = theRobotPose.inverse() * teammate.pose.translation;
        const Angle range = std::max(1_deg, static_cast<Angle>(std::asin(std::min(1.f, 100.f / position.norm()))));
        const Angle angle = position.angle();
        RING_SECTOR3D("representation:GlobalTeammatesModel:sectors", Vector3f(0, 0, 3.f), angle - range, angle + range, 140.f, 160.f,
                      teammate.playerNumber == theGameState.ownTeam.goalkeeperNumber ? goalkeeperColor : fieldPlayerColor);
      }
    }
  }
}
