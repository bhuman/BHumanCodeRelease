/**
 * @file Dive.cpp
 *
 * This file implements the Dive skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) Dive,
       args((MotionRequest::Dive::Request) request))
{
  theMotionRequest.motion = MotionRequest::dive;
  theMotionRequest.diveRequest = request;
  theLibCheck.inc(LibCheck::motionRequest);

  initial_state(execute)
  {
    transition
    {
      const KeyframeMotionRequest keyframeMotionRequest = KeyframeMotionRequest::fromDiveRequest(request);
      if(theMotionInfo.isKeyframeMotion(keyframeMotionRequest.keyframeMotion, keyframeMotionRequest.mirror))
        goto done;
    }
  }

  target_state(done) {}
}
