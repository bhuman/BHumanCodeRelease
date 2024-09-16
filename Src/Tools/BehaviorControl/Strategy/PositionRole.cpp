/**
 * @file PositionRole.cpp
 *
 * This file implements a base class for position roles.
 *
 * @author Arne Hasselbring
 */

#include "PositionRole.h"
#include "Agent.h"
#include "Platform/BHAssert.h"
#include <cmath>

PositionRole::Type PositionRole::fromPosition(Tactic::Position::Type type)
{
  switch(type)
  {
    case Tactic::Position::goalkeeper:
      return goalkeeper;
    case Tactic::Position::attackingGoalkeeper:
      return attackingGoalkeeper;
    case Tactic::Position::defender:
    case Tactic::Position::defenderL:
    case Tactic::Position::defenderR:
      return defender;
    case Tactic::Position::midfielder:
    case Tactic::Position::midfielderM:
    case Tactic::Position::midfielderL:
    case Tactic::Position::midfielderR:
      return midfielder;
    case Tactic::Position::forward:
    case Tactic::Position::forwardM:
    case Tactic::Position::forwardL:
    case Tactic::Position::forwardR:
      return forward;
    case Tactic::Position::sacPasser:
      return sacPasser;
  }
  FAIL("Unknown position.");
  return PositionRole::midfielder;
}

PositionRole::Side PositionRole::sideFromPosition(Tactic::Position::Type type)
{
  switch(type)
  {
    case Tactic::Position::goalkeeper:
    case Tactic::Position::attackingGoalkeeper:
    case Tactic::Position::defender:
    case Tactic::Position::midfielder:
    case Tactic::Position::forward:
    case Tactic::Position::sacPasser:
      return unspecified;
    case Tactic::Position::midfielderM:
    case Tactic::Position::forwardM:
      return center;
    case Tactic::Position::defenderL:
    case Tactic::Position::midfielderL:
    case Tactic::Position::forwardL:
      return left;
    case Tactic::Position::defenderR:
    case Tactic::Position::midfielderR:
    case Tactic::Position::forwardR:
      return right;
  }
  FAIL("Unknown position.");
  return unspecified;
}

Pose2f PositionRole::position(Side, const Pose2f& basePose, const std::vector<Vector2f>&, const Agent&, const Agents&)
{
  return basePose;
}

Pose2f PositionRole::tolerance() const
{
  return Pose2f(5_deg, 150.f, 150.f);
}

bool PositionRole::shouldStart(const Pose2f& target) const
{
  const Pose2f error = target.inverse() * theRobotPose;
  const Pose2f max = tolerance();
  return std::abs(error.rotation) > max.rotation ||
         std::abs(error.translation.x()) > max.translation.x() ||
         std::abs(error.translation.y()) > max.translation.y();
}

bool PositionRole::shouldStop(const Pose2f&) const
{
  return false;
}

SkillRequest PositionRole::execute(const Agent& self, const Agents& teammates)
{
  Pose2f target = position(sideFromPosition(self.position), self.basePose, self.baseArea, self, teammates);
  if(theGameState.isKickOff() && self.acceptedSetPlay != SetPlay::none)
    target = self.basePose;

  // Theoretically we should use shouldStop and shouldStart of the base class when the base pose is used
  if(theMotionInfo.executedPhase != MotionPhase::stand ? !shouldStop(target) : shouldStart(target))
    return SkillRequest::Builder::walkTo(target);
  else
    return SkillRequest::Builder::stand();
}
