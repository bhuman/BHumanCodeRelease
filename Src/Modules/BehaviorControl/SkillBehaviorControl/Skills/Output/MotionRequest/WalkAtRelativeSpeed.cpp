/**
 * @file WalkAtRelativeSpeed.cpp
 *
 * This file implements the WalkAtRelativeSpeed skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) WalkAtRelativeSpeed,
       args((const Pose2f&) speed,
            (bool) energySavingWalk))
{
  theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
  theMotionRequest.walkSpeed = speed;
  theMotionRequest.energySavingWalk = energySavingWalk;
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
