/**
 * @file PassTarget.cpp
 *
 * This file implements the implementation of the PassTarget skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"

SKILL_IMPLEMENTATION(PassTargetImpl,
{,
  IMPLEMENTS(PassTarget),
  REQUIRES(LibCheck),
  MODIFIES(BehaviorStatus),
});

class PassTargetImpl : public PassTargetImplBase
{
  void execute(const PassTarget& p) override
  {
    theBehaviorStatus.passTarget = p.passTarget;
    theBehaviorStatus.shootingTo = p.ballTarget;
    theLibCheck.inc(LibCheck::passTarget);
  }
};

MAKE_SKILL_IMPLEMENTATION(PassTargetImpl);
