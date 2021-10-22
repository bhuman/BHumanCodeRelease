/**
 * @file Dribble.cpp
 *
 * This file implements the implementation of the Dribble skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(DribbleImpl,
{,
  IMPLEMENTS(Dribble),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class DribbleImpl : public DribbleImplBase
{
  void execute(const Dribble& p) override
  {
    theMotionRequest.motion = MotionRequest::dribble;
    theMotionRequest.walkSpeed = p.speed;
    theMotionRequest.obstacleAvoidance = p.obstacleAvoidance;
    theMotionRequest.targetDirection = p.targetDirection;
    theMotionRequest.directionPrecision = p.directionPrecision;
    theMotionRequest.alignPrecisely = p.alignPrecisely;
    theMotionRequest.kickPower = p.kickPower;
    theMotionRequest.turnKickAllowed = p.turnKickAllowed;
    theMotionRequest.preStepAllowed = p.preStepAllowed;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const Dribble&) const override
  {
    return theMotionInfo.executedPhase == MotionPhase::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(DribbleImpl);
