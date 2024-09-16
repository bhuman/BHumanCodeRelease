/**
 * @file ReplayWalk.cpp
 *
 * This file implements the ReplayWalk skill.
 *
 * @author Philip Reichenberg
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) ReplayWalk)
{
  theMotionRequest.motion = MotionRequest::replayWalk;
  theMotionRequest.standHigh = false;
  theMotionRequest.ballEstimate.position.x() = 1000.f; // To make sure the walk step adjustment ignores the ball position
  theLibCheck.inc(LibCheck::motionRequest);

  initial_state(execute)
  {
    transition
    {
      if(theMotionInfo.executedPhase == MotionPhase::replayWalk)
        goto done;
    }
  }

  target_state(done) {}
}
