/**
 * @file RecordTargetAndSpeed.cpp
 *
 * This skill records the walkingTo target and desired speed in the BehaviorStatus.
 *
 * @author Lukas Plecher
 */

#include "Platform/BHAssert.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"

SKILL_IMPLEMENTATION(RecordTargetAndSpeedImpl,
{,
  IMPLEMENTS(RecordTargetAndSpeed),
  REQUIRES(WalkingEngineOutput),
  MODIFIES(BehaviorStatus),
});

class RecordTargetAndSpeedImpl : public RecordTargetAndSpeedImplBase
{
  void execute(const RecordTargetAndSpeed& p) override
  {
    ASSERT(p.speed >= 0.f);
    ASSERT(p.speed <= 1.f);
    theBehaviorStatus.walkingTo = p.target;
    theBehaviorStatus.speed = p.speed * theWalkingEngineOutput.maxSpeed.translation.x();
  }
};

MAKE_SKILL_IMPLEMENTATION(RecordTargetAndSpeedImpl);
