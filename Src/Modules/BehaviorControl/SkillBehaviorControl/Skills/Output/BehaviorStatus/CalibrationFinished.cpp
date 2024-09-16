/**
 * @file CalibrationFinished.cpp
 *
 * This file implements the CalibrationFinished skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) CalibrationFinished)
{
  initial_state(execute)
  {
    action
    {
      theBehaviorStatus.calibrationFinished = true;
      theLibCheck.inc(LibCheck::calibrationFinished);
    }
  }
}
