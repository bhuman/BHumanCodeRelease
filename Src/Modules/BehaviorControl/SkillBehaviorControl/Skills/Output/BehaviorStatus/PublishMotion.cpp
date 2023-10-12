/**
 * @file PublishMotion.cpp
 *
 * This skill records the walkingTo target and desired speed in the BehaviorStatus
 * to be published to teammates.
 *
 * @author Lukas Plecher
 */

#include "Platform/BHAssert.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"

SKILL_IMPLEMENTATION(PublishMotionImpl,
{,
  IMPLEMENTS(PublishMotion),
  REQUIRES(WalkingEngineOutput),
  MODIFIES(BehaviorStatus),
});

class PublishMotionImpl : public PublishMotionImplBase
{
  void execute(const PublishMotion& p) override
  {
    ASSERT(p.speed.translation.x() >= 0.f);
    ASSERT(p.speed.translation.x() <= 1.f);
    theBehaviorStatus.walkingTo = p.target;
    theBehaviorStatus.speed = p.speed.translation.x() * theWalkingEngineOutput.maxSpeed.translation.x();
  }
};

MAKE_SKILL_IMPLEMENTATION(PublishMotionImpl);
