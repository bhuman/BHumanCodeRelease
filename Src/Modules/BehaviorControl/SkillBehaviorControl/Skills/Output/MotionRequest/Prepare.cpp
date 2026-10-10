/**
 * @file Prepare.cpp
 *
 * This file implements the passive stand skill.
 *
 * @author Philip Reichenberg
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) Prepare)
{
  theMotionRequest.motion = MotionRequest::prepare;
  theLibCheck.inc(LibCheck::motionRequest);

  initial_state(execute)
  {
    transition
    {
      if(theMotionInfo.executedPhase == MotionPhase::prepare)
        goto done;
    }
  }

  target_state(done) {}
}
