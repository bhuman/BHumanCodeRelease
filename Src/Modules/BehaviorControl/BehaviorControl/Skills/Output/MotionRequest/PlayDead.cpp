/**
 * @file PlayDead.cpp
 *
 * This file implements the implementation of the PlayDead skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(PlayDeadImpl,
{,
  IMPLEMENTS(PlayDead),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class PlayDeadImpl : public PlayDeadImplBase
{
  void execute(const PlayDead&) override
  {
    theMotionRequest.motion = MotionRequest::playDead;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const PlayDead&) const override
  {
    return theMotionInfo.executedPhase == MotionPhase::playDead;
  }
};

MAKE_SKILL_IMPLEMENTATION(PlayDeadImpl);
