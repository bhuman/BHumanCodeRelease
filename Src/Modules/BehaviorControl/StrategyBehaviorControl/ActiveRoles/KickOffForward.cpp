/**
 * @file KickOffForward.cpp
 *
 * This file implements a behavior to walk into the opposing half after the kick off.
 *
 * @author Thade Struckhoff
 */

#include "KickOffForward.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"
#include "Tools/BehaviorControl/Strategy/Tactic.h"

SkillRequest KickOffForward::execute(const Agent& self, const Agents&)
{
  const Vector2f ballPosition = theFieldBall.recentBallPositionOnField(); // the position of the ball on the field
  Vector2f targetPosition;

  if(Tactic::Position::forwardL == self.position || Tactic::Position::forwardR == self.position)
    targetPosition.x() = OuterForwardX;
  else
    targetPosition.x() = centralForwardX;
  targetPosition.y() = self.pose.translation.y();
  return SkillRequest::Builder::walkTo(Pose2f((ballPosition - targetPosition).angle(), targetPosition));
}
