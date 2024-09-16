/**
 * @file PublishMotion.cpp
 *
 * This skill records the walkingTo target and desired speed in the BehaviorStatus
 * to be published to teammates.
 *
 * @author Lukas Plecher
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) PublishMotion,
       args((const Vector2f) target,
            (const Pose2f&) speed))
{
  initial_state(execute)
  {
    action
    {
      ASSERT(speed.translation.x() >= 0.f);
      ASSERT(speed.translation.x() <= 1.f);
      theBehaviorStatus.walkingTo = target;
      theBehaviorStatus.speed = speed.translation.x() * theWalkingEngineOutput.maxSpeed.translation.x();
    }
  }
}
