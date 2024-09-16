/**
 * @file Special.cpp
 *
 * This file implements the Special skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) Special,
       args((MotionRequest::Special::Request) request))
{
  theMotionRequest.motion = MotionRequest::special;
  theMotionRequest.specialRequest = request;
  theLibCheck.inc(LibCheck::motionRequest);

  initial_state(execute)
  {
    transition
    {
      const KeyframeMotionRequest keyframeMotionRequest = KeyframeMotionRequest::fromSpecialRequest(request);
      if(theMotionInfo.isKeyframeMotion(keyframeMotionRequest.keyframeMotion, keyframeMotionRequest.mirror))
        goto done;
    }
  }

  target_state(done) {}
}
