/**
 * @file CalibrationFinished.cpp
 *
 * This file implements the implementation of the CalibrationFinished skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"

SKILL_IMPLEMENTATION(CalibrationFinishedImpl,
{,
  IMPLEMENTS(CalibrationFinished),
  REQUIRES(LibCheck),
  MODIFIES(BehaviorStatus),
});

class CalibrationFinishedImpl : public CalibrationFinishedImplBase
{
  void execute(const CalibrationFinished&) override
  {
    theBehaviorStatus.calibrationFinished = true;
    theLibCheck.inc(LibCheck::calibrationFinished);
  }
};

MAKE_SKILL_IMPLEMENTATION(CalibrationFinishedImpl);
