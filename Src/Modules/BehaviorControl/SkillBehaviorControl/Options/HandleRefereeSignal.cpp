/**
 * @file HandleRefereeSignal.h
 *
 * This file defines an option that handles the detection of referee signals
 * in the standby state and during kick-ins. A robot only tries to look at
 * the referee if it stands inside a ring segment surrounding the referee
 * position, when a state starts that requires detecting a referee signal.
 * The detection stops when any robot on the team detects an expected
 * referee signal, a timeout is reached, or the state ends. In case of a
 * kick-in, the robot will turn in the direction of the referee if just
 * turning the head is not sufficient.
 *
 * @author Thomas Röfer
 */

#include "SkillBehaviorControl.h"

/**
 * The detection of the referee signal. It only becomes active if the robot is
 * in the right place.
 */
option((SkillBehaviorControl) HandleRefereeSignal,
       defs((float)(2400.f) upperImageBorderAtHeight, /**< The height to look at (in mm). */
            (float)(2600.f) upperImageBorderAtHeightStandby, /**< The height to look at in standby (in mm). */
            (float)(515.f) assumedCameraHeight, /**< The assumed height of the camera above ground (in mm). */
            (Rangef)({2000.f, 6000.f}) distanceRange, /**< The distance range for this option to become active during kick-in. */
            (Rangea)({60_deg, 120_deg}) bearingRange, /**< The bearing range to the referee for this option to become active. */
            (int)(6000) kickInWaitTime, /**< How long to unsuccessfully look at referee during kick-in. */
            (Angle)(45_deg) maxHeadTurn, /**< Maximum head rotation before the body has to be turned. */
            (Angle)(2_deg) turnTolerance)) /**< Accepted tolerance when reaching the required body rotation. */
{
  const Vector2f refereeOnField(theFieldDimensions.xPosHalfwayLine,
                                (theFieldDimensions.yPosLeftTouchline + theFieldDimensions.yPosLeftFieldBorder) / 2.f
                                * (theGameState.leftHandTeam ? 1 : -1));
  const Vector2f refereeOffsetOnField = refereeOnField - theRobotPose.translation;
  const Vector2f refereeOffsetRelative = theRobotPose.inverse() * refereeOnField;
  const float refereeDistance = refereeOffsetOnField.norm();

  // This assumes that both cameras have roughly the same opening angles.
  const float lookAtHeight = std::tan(std::atan2((theGameState.state == GameState::standby || theGameState.state == GameState::beforeHalf ||
                                                  theGameState.state == GameState::timeout
                                                  ? upperImageBorderAtHeightStandby : upperImageBorderAtHeight)
                                                 - assumedCameraHeight, refereeDistance)
                                      - theCameraInfo.openingAngleHeight * 0.5f) * refereeDistance + assumedCameraHeight;

  const auto refereeSignalDetected = [&](const RefereeSignal::Signal signal)
  {
    if(theRefereeSignal.signal == signal && theRefereeSignal.timeWhenDetected >= theGameState.timeWhenStateStarted)
      return true;
    else
      for(const Teammate& teammate : theTeamData.teammates)
        if(teammate.theRefereeSignal.signal == signal && teammate.theRefereeSignal.timeWhenDetected >= theGameState.timeWhenStateStarted)
          return true;
    return false;
  };

  common_transition
  {
    if((theGameState.state != GameState::standby && theGameState.state != GameState::beforeHalf && theGameState.state != GameState::timeout && !theGameState.isKickIn())
       || (theGameState.state == GameState::standby && refereeSignalDetected(RefereeSignal::ready))
       || (theGameState.isKickIn() && (theStrategyStatus.role == ActiveRole::toRole(ActiveRole::freeKickWall)
                                       || theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) > kickInWaitTime
                                       || refereeSignalDetected(RefereeSignal::kickInLeft)
                                       || refereeSignalDetected(RefereeSignal::kickInRight))))
      goto inactive;
  }

  initial_state(inactive)
  {
    transition
    {
      DEBUG_RESPONSE_ONCE("option:HandleRefereeSignal:now")
        goto turnToReferee;
      if(theGameState.gameControllerActive && bearingRange.isInside(std::abs(refereeOffsetOnField.angle())))
      {
        if(theGameState.state == GameState::standby || theGameState.state == GameState::beforeHalf || theGameState.state == GameState::timeout)
          goto lookAtReferee;
        else if(!theGameState.kickingTeamKnown && theGameState.isKickIn() && distanceRange.isInside(refereeDistance))
          goto turnToReferee;
      }
    }
  }

  state(turnToReferee)
  {
    transition
    {
      if(std::abs(refereeOffsetRelative.angle()) < maxHeadTurn)
        goto lookAtReferee;
    }
    action
    {
      const Angle lookDir = Rangea(-maxHeadTurn, maxHeadTurn).clamped(refereeOffsetRelative.angle());
      const Vector2f lookOffset = Pose2f(lookDir) * Vector2f(refereeOffsetRelative.norm(), 0.f);
      LookAtPoint({.target = {lookOffset.x(), lookOffset.y(), lookAtHeight},
                   .camera = HeadMotionRequest::upperCamera});
      const Angle rotationDiff = refereeOffsetRelative.angle();
      WalkToPose({.target = {std::max(0.f, std::abs(rotationDiff) - maxHeadTurn + turnTolerance) * sgn(rotationDiff)}});
    }
  }

  state(lookAtReferee)
  {
    action
    {
      LookAtPoint({.target = {refereeOffsetRelative.x(), refereeOffsetRelative.y(), lookAtHeight},
                   .camera = HeadMotionRequest::upperCamera});
      Stand({.high = (theGameState.state == GameState::standby || theGameState.state == GameState::beforeHalf || theGameState.state ==
                      GameState::timeout)});
      theRefereeDetectionRequest.detectReferee = true;
    }
  }
}
