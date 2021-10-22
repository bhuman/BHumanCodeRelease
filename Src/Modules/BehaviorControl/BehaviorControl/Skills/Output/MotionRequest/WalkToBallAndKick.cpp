/**
 * @file WalkToBallAndKick.cpp
 *
 * This file implements the implementation of the WalkToBallAndKick skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(WalkToBallAndKickImpl,
{,
  IMPLEMENTS(WalkToBallAndKick),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class WalkToBallAndKickImpl : public WalkToBallAndKickImplBase
{
  void execute(const WalkToBallAndKick& p) override
  {
    theMotionRequest.motion = MotionRequest::walkToBallAndKick;
    theMotionRequest.walkSpeed = p.speed;
    theMotionRequest.obstacleAvoidance = p.obstacleAvoidance;
    theMotionRequest.targetDirection = p.targetDirection;
    theMotionRequest.directionPrecision = p.directionPrecision;
    theMotionRequest.kickType = p.kickType;
    theMotionRequest.kickPower = p.kickPower;
    theMotionRequest.alignPrecisely = p.alignPrecisely;
    theMotionRequest.preStepAllowed = p.preStepAllowed;
    theMotionRequest.turnKickAllowed = p.turnKickAllowed;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const WalkToBallAndKick& p) const override
  {
    return (p._context.lastFrame == SkillRegistry::theInstance->lastFrameTime || p._context.lastFrame == SkillRegistry::theInstance->currentFrameTime) && theMotionInfo.lastKickTimestamp > lastKickTimestamp;
  }

  void postProcess(const WalkToBallAndKick&) override
  {
    lastKickTimestamp = theMotionInfo.lastKickTimestamp;
  }

  using SkillImplementationInterface::postProcess;

  unsigned lastKickTimestamp = 0; /**< The value of the kick timestamp when this skill was entered. */
};

MAKE_SKILL_IMPLEMENTATION(WalkToBallAndKickImpl);
