/**
 * @file WalkAtRelativeSpeed.cpp
 *
 * This file implements the WalkAtRelativeSpeed skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) WalkAtRelativeSpeed,
       args((const Pose2f&) speed))
{
  theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
  theMotionRequest.walkSpeed = speed;
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
