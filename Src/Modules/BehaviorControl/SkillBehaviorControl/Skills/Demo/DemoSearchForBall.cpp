/**
 * @file DemoSearchForBall.cpp
 *
 * This file implements the DemoSearchForBall skill.
 *
 * @author Philip Reichenberg
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) DemoSearchForBall,
       defs((Angle)(75_deg) maxTurnSpeed, /**< Used turn speed to rotate while walking. */
            (Angle)(75_deg) maxTurnStep, /**< Next relative target rotation when turning. */
            (Angle)(15_deg) targetRotationDeviation, /**< When the target rotation and RobotPose rotation deviate less than this, then we turned enough. */
            (int)(2000) maxHeadSideSearch, /**< Look to the side for this time duration. */
            (int)(1000) maxHeadForwardSearch, /**< Look forward for this time duration. */
            (int)(3000) maxTurnDuration, /**< Turn around for max this time duration. This is needed, if the underground is very slippery. */
            (Angle)(50_deg) panAngle, /**< Used pan angle when searching. */
            (Angle)(20_deg) tiltAngle), /**< Used tilt angle when searching. */
       vars((Angle)(0_deg) target, /**< Next target rotation, when turning. */
            (bool)(theFieldBall.positionRelative.y() > 0) isSearchLeft)) /**< Is search direction left from us? */
{
  // Look to the search direction, wait, look opposite, wait, start turning
  initial_state(lookForward)
  {
    transition
    {
      if(state_time > maxHeadForwardSearch)
        goto lookLeftAndRight;
    }
    action
    {
      Stand();
      LookAtAngles({.pan = 0_deg,
                    .tilt = tiltAngle});
    }
  }

  state(lookLeftAndRight)
  {
    transition
    {
      if(state_time > 2 * maxHeadSideSearch)
        goto turn;
    }
    action
    {
      Stand();
      const bool lookLeft = (isSearchLeft && state_time < maxHeadSideSearch) ||
                            (!isSearchLeft && state_time >= maxHeadSideSearch);
      LookAtAngles({.pan = lookLeft ? panAngle : -panAngle,
                    .tilt = tiltAngle});
    }
  }

  state(turn)
  {
    transition
    {
      if(theRobotPose.rotation.diffAbs(target) < targetRotationDeviation || state_time > maxTurnDuration)
        goto lookForward;
    }
    action
    {
      if(state_time == 0)
      {
        target = theRobotPose.rotation;
        target += isSearchLeft ? maxTurnStep : -maxTurnStep;
      }
      LookAtAngles({.pan = 0.f,
                    .tilt = tiltAngle});
      WalkAtAbsoluteSpeed({.speed = {isSearchLeft ? maxTurnSpeed : -maxTurnSpeed}});
    }
  }
}
