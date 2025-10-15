/**
 * @file HandleRefereeDatasetGeneration.h
 *
 * This files defines an option that lets the robot look at the t-junction as if there was a referee.
 * This option is a helper file to generate lots of images with referee gestures and therefore
 * to generate a dataset faster. The robots shall not move (unless they are orientated wrong) and only move their head towards the t-junction.
 *
 * Note that the parameters should be the same as for the normal referee detection.
 *
 * @author Philip Reichenberg
 */

#include "SkillBehaviorControl.h"

/**
   * The detection of the referee signal. It only becomes active if the robot is
   * in the right place.
   */
option((SkillBehaviorControl) HandleRefereeDatasetGeneration,
       defs((float)(2400.f) upperImageBorderAtHeight, /**< The height to look at (in mm). */
            (float)(515.f) assumedCameraHeight, /**< The assumed height of the camera above ground (in mm). */
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
  const float lookAtHeight = std::tan(std::atan2(upperImageBorderAtHeight - assumedCameraHeight, refereeDistance)
                                      - theCameraInfo.openingAngleHeight * 0.5f) * refereeDistance + assumedCameraHeight;

  initial_state(inactive)
  {
    transition
    {
      goto turnToReferee;
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
      Stand({.high = true});
      theRefereeDetectionRequest.detectReferee = true;
    }
  }
}
