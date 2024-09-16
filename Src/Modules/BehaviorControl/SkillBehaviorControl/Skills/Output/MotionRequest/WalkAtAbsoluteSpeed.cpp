/**
 * @file WalkAtAbsoluteSpeed.cpp
 *
 * This file implements the WalkAtAbsoluteSpeed skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) WalkAtAbsoluteSpeed,
       args((const Pose2f&) speed))
{
  theMotionRequest.motion = MotionRequest::walkAtAbsoluteSpeed;
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
