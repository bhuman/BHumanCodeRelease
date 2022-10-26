/**
 * @file Activity.cpp
 *
 * This file implements the implementation of the Activity skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"

SKILL_IMPLEMENTATION(ActivityImpl,
{,
  IMPLEMENTS(Activity),
  REQUIRES(LibCheck),
  MODIFIES(BehaviorStatus),
});

class ActivityImpl : public ActivityImplBase
{
  void execute(const Activity& p) override
  {
    theBehaviorStatus.activity = p.activity;
    theLibCheck.inc(LibCheck::activity);
  }
};

MAKE_SKILL_IMPLEMENTATION(ActivityImpl);
