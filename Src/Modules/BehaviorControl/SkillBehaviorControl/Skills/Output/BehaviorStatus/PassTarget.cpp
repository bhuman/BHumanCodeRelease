/**
 * @file PassTarget.cpp
 *
 * This file implements the PassTarget skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) PassTarget,
       args((int) passTarget,
            (const Vector2f&) ballTarget))
{
  initial_state(execute)
  {
    action
    {
      theBehaviorStatus.passTarget = passTarget;
      theBehaviorStatus.shootingTo = ballTarget;
      theLibCheck.inc(LibCheck::passTarget);
    }
  }
}
