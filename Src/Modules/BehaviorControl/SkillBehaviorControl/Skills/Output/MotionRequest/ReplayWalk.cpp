/**
 * @file ReplayWalk.cpp
 *
 * This file implements the implementation of the ReplayWalk skill.
 *
 * @author Philip Reichenberg
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(ReplayWalkImpl,
{,
  IMPLEMENTS(ReplayWalk),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class ReplayWalkImpl : public ReplayWalkImplBase
{
  void execute(const ReplayWalk&) override
  {
    theMotionRequest.motion = MotionRequest::replayWalk;
    theMotionRequest.standHigh = false;
    theMotionRequest.ballEstimate.position.x() = 1000.f; // To make sure the walk step adjustment ignores the ball position
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const ReplayWalk&) const override
  {
    return theMotionInfo.executedPhase == MotionPhase::replayWalk;
  }
};

MAKE_SKILL_IMPLEMENTATION(ReplayWalkImpl);
