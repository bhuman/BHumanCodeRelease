/**
 * @file TurnAngle.cpp
 *
 * This file implements the TurnAngle skill
 * that uses the OdometryData to keep track of the orientation
 * (instead of the RobotPose, which means that localization jumps
 * do not affect this skill).
 *
 * @author Arne Hasselbring (the actual behavior is older)
 */

#include "SkillBehaviorControl.h"
#include <cmath>

option((SkillBehaviorControl) TurnAngle,
       args((Angle) angle,
            (Angle) margin),
       vars((Angle)(theOdometryData.rotation) startRotation)) /**< The global rotation of the robot when the turn started. */
{
  initial_state(execute)
  {
    transition
    {
      if(std::abs(Angle::normalize(startRotation + angle - theOdometryData.rotation)) < margin)
        goto done;
    }
    action
    {
      WalkToPoint({.target = {Angle::normalize(startRotation + angle - theOdometryData.rotation)},
                   .reduceWalkingSpeed = ReduceWalkSpeedType::noChange});
    }
  }

  target_state(done)
  {
    action
    {
      WalkToPoint({.target = {Angle::normalize(startRotation + angle - theOdometryData.rotation)},
                   .reduceWalkingSpeed = ReduceWalkSpeedType::noChange});
    }
  }
}
