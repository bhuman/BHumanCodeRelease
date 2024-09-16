/**
 * @file TurnToPoint.cpp
 *
 * This file implements the TurnToPoint skill.
 *
 * @author Nicole Schrader
 */

#include "SkillBehaviorControl.h"
#include <cmath>

option((SkillBehaviorControl) TurnToPoint,
       args((const Vector2f&) target,
            (Angle) margin))
{
  initial_state(execute)
  {
    transition
    {
      if(action_done)
        goto done;
    }
    action
    {
      TurnAngle({.angle = target.angle(),
                 .margin = margin});
    }
  }

  target_state(done)
  {
    action
    {
      TurnAngle({.angle = target.angle(),
                 .margin = margin});
    }
  }
}
