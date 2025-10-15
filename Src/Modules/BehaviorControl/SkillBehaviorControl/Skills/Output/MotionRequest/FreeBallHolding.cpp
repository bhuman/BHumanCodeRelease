/**
 * @file FreeBallHolding.cpp
 *
 * This file implements the FreeBallHolding skill.
 *
 * @author Harm Thordsen
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) FreeBallHolding,
       args((Angle) targetDirection))
{
  theMotionRequest.motion = MotionRequest::freeBallHolding;
  theMotionRequest.walkTarget = Pose2f();
  theMotionRequest.walkSpeed = Pose2f(1.f, 1.f, 1.f);
  theMotionRequest.targetDirection = targetDirection;
  theMotionRequest.energySavingWalk = false;
  theLibCheck.inc(LibCheck::motionRequest);

  initial_state(execute)
  {
    transition
    {
      if(theMotionInfo.executedPhase == MotionPhase::walk)
        goto done;
    }
  }

  target_state(done) {}
}
