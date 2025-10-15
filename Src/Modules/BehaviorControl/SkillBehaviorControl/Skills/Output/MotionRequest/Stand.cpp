/**
 * @file Stand.cpp
 *
 * This file implements the Stand skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) Stand,
       args((bool) high,
            (bool) energySavingWalk))
{
  theMotionRequest.motion = MotionRequest::stand;
  theMotionRequest.standHigh = high;
  theMotionRequest.energySavingWalk = energySavingWalk;
  theLibCheck.inc(LibCheck::motionRequest);

  initial_state(execute)
  {
    transition
    {
      if(theMotionInfo.executedPhase == MotionPhase::stand)
        goto done;
    }
  }

  target_state(done) {}
}
