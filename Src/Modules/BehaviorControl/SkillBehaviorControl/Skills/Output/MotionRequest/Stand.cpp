/**
 * @file Stand.cpp
 *
 * This file implements the Stand skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) Stand,
       args((bool) high))
{
  theMotionRequest.motion = MotionRequest::stand;
  theMotionRequest.standHigh = high;
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
