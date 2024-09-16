/**
 * @file HandleRefereeSignal.h
 *
 * This file defines an option that handles the referee detection in the
 * initial state. So far, this only means that a robot looks at the referee
 * when suitable.
 *
 * @author Thomas Röfer
 */

#include "SkillBehaviorControl.h"

/**
 * The detection of the referee signal. In only becomes active if the robot is
 * in the right place.
 */
option((SkillBehaviorControl) HandleRefereeSignal,
       defs((float)(100.f) lookAtHeight, /**< The height to look at (in mm). */
            (Angle)(30_deg) maxBearing)) /**< The maximum bearing to the referee this option becomes active. */
{
  const Vector2f refereeOnField(theFieldDimensions.xPosHalfwayLine,
                                (theFieldDimensions.yPosLeftTouchline + theFieldDimensions.yPosLeftFieldBorder) / 2.f
                                * (theGameState.leftHandTeam ? 1 : -1));
  const Vector2f refereeOffset = theRobotPose.inverse() * refereeOnField;

  common_transition
  {
    if(theGameState.state != GameState::standby || std::abs(refereeOffset.angle()) >= maxBearing)
      goto inactive;
  }

  initial_state(inactive)
  {
    transition
    {
      if(theGameState.gameControllerActive
         && theGameState.state == GameState::standby
         && std::abs(refereeOffset.angle()) < maxBearing)
        goto lookAtReferee;
    }
  }

  state(lookAtReferee)
  {
    action
    {
      Stand({.high = true});
      LookAtPoint({.target = {refereeOffset.x(), refereeOffset.y(), lookAtHeight},
                   .camera = HeadMotionRequest::upperCamera});
      theOptionalImageRequest.sendImage = true;
    }
  }
}
