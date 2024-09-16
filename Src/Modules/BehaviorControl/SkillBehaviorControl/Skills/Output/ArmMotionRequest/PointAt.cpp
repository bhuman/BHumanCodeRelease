/**
 * @file PointAt.cpp
 *
 * This file implements the PointAt skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) PointAt,
       args((const Vector3f&) localPoint,
            (Arms::Arm) arm))
{
  initial_state(initial)
  {
    transition
    {
      if(arm == Arms::left || (arm == Arms::numOfArms && localPoint.y() > 0.f))
        goto leftArm;
      else
        goto rightArm;
    }
  }

  state(leftArm)
  {
    transition
    {
      if(arm == Arms::numOfArms && localPoint.y() < -50.f)
        goto rightArm;
    }
    action
    {
      theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::pointAt;
      theArmMotionRequest.pointToPointAt = localPoint;
    }
  }

  state(rightArm)
  {
    transition
    {
      if(arm == Arms::numOfArms && localPoint.y() > 50.f)
        goto leftArm;
    }
    action
    {
      theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::pointAt;
      theArmMotionRequest.pointToPointAt = localPoint;
    }
  }
}
