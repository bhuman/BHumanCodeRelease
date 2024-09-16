/**
 * @file StrategyStatus.cpp
 *
 * This file declares a representation of the status of the strategy.
 *
 * @author Philip Reichenberg
 */

#include "StrategyStatus.h"
#include "Debugging/DebugDrawings3D.h"

void StrategyStatus::draw() const
{
  DEBUG_DRAWING3D("representation:StrategyStatus:playBall", "field")
  {
    if(role == ActiveRole::toRole(ActiveRole::playBall))
    {
      const RobotPose& theRobotPose = static_cast<RobotPose&>(Blackboard::getInstance()["RobotPose"]);
      const Pose3f point = Pose3f(theRobotPose.translation.x(), theRobotPose.translation.y(), 0.f);
      CIRCLE3D("representation:StrategyStatus:playBall", point, 100.f, 5, ColorRGBA::cyan);
    }
  }
}
