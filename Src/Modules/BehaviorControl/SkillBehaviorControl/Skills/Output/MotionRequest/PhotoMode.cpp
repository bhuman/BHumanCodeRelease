/**
 * @file PhotoMode.cpp
 *
 * This file implements the PhotoMode skill.
 *
 * @author Philip Reichenberg
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) PhotoMode)
{
  theMotionRequest.motion = MotionRequest::photoMode;
  theLibCheck.inc(LibCheck::motionRequest);

  initial_state(execute)
  {
    transition
    {
      if(theMotionInfo.executedPhase == MotionPhase::photoMode)
        goto done;
    }
  }

  target_state(done) {}
}
