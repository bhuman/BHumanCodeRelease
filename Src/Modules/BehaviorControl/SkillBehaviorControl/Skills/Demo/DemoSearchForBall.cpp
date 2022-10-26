/**
 * @file DemoSearchForBall.cpp
 *
 * This file implements an implementation of the DemoSearchForBall skill.
 *
 * @author Philip Reichenberg
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(DemoSearchForBallImpl,
{,
  IMPLEMENTS(DemoSearchForBall),
  CALLS(LookAtAngles),
  CALLS(Stand),
  CALLS(WalkAtAbsoluteSpeed),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (Angle)(75_deg) maxTurnSpeed, /**< Used turn speed to rotate while walking. */
    (Angle)(75_deg) maxTurnStep, /**< Next relative target rotation when turning. */
    (Angle)(15_deg) targetRotationDeviation, /**< When the target rotation and RobotPose rotation deviate less than this, then we turned enough. */
    (int)(2000) maxHeadSideSearch, /**< Look to the side for this time duration. */
    (int)(1000) maxHeadForwardSearch, /**< Look forward for this time duration. */
    (int)(3000) maxTurnDuration, /**< Turn around for max this time duration. This is needed, if the underground is very slippery. */
    (Angle)(50_deg) panAngle, /**< Used pan angle when searching. */
    (Angle)(20_deg) tiltAngle, /**< Used tilt angle when searching. */
  }),
});

class DemoSearchForBallImpl : public DemoSearchForBallImplBase
{
  option(DemoSearchForBall)
  {
    // Init the search direction
    initial_state(initial)
    {
      transition
      {
        initSearchDirection();
        goto lookLeftAndRight;
      }
    }

    // Look to the search direction, wait, look opposite, wait, start turning
    state(lookLeftAndRight)
    {
      transition
      {
        if(state_time > 2 * maxHeadSideSearch + maxHeadForwardSearch)
          goto turn;
      }
      action
      {
        theStandSkill();
        if(state_time < maxHeadForwardSearch)
          theLookAtAnglesSkill({.pan = 0_deg,
                                .tilt = tiltAngle});
        else
        {
          const bool lookLeft = (isSearchLeft && state_time < maxHeadSideSearch + maxHeadForwardSearch) ||
                                (!isSearchLeft && state_time >= maxHeadSideSearch + maxHeadForwardSearch);
          theLookAtAnglesSkill({.pan = lookLeft ? panAngle : -panAngle,
                                .tilt = tiltAngle});
        }
      }
    }

    state(turn)
    {
      transition
      {
        if(theRobotPose.rotation.diffAbs(target) < targetRotationDeviation || state_time > maxTurnDuration)
          goto lookLeftAndRight;
      }
      action
      {
        if(state_time == 0)
        {
          target = theRobotPose.rotation;
          target += isSearchLeft ? maxTurnStep : -maxTurnStep;
        }
        theLookAtAnglesSkill({.pan = 0.f,
                              .tilt = tiltAngle});
        theWalkAtAbsoluteSpeedSkill({.speed = {isSearchLeft ? maxTurnSpeed : -maxTurnSpeed}});
      }
    }
  }

private:

  void initSearchDirection()
  {
    isSearchLeft = theFieldBall.positionRelative.y() > 0;
  }

  Angle target = 0_deg;  /**< Next target rotation, when turning. */
  bool isSearchLeft = false; /**< Is search direction left from us? */
};

MAKE_SKILL_IMPLEMENTATION(DemoSearchForBallImpl);
