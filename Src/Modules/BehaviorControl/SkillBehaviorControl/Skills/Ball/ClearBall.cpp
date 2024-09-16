/**
 * @file ClearBall.cpp
 *
 * This file defines an implementation of the ClearBall skill.
 *
 * @author Nico Holsten
 */

#include "SkillBehaviorControl.h"
#include "Platform/BHAssert.h"

option((SkillBehaviorControl) ClearBall)
{
  initial_state(initial)
  {
    action
    {
      ASSERT(theClearTarget.getKickType() != KickInfo::numOfKickTypes);
      GoToBallAndKick({.targetDirection = theClearTarget.getAngle(),
                       .kickType = theClearTarget.getKickType(),
                       .preStepType = PreStepType::forced});
    }
  }
}
