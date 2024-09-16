/**
 * @file InterceptBallProvider.h
 *
 * This file implements a module that intercepts the ball
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/InterceptBallGenerator.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(InterceptBallProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FieldInterceptBall),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(KickInfo),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkGenerator),
  PROVIDES(InterceptBallGenerator),
  LOADS_PARAMETERS(
  {,
    (Rangef) maxYShift, /**< Optimize interception point. */
    (Rangef) maxXStep, /**< Optimize interception point. */
    (Angle) maxRotationStep, /**< For intercepting, use this rotation threshold. */
    (Angle) maxBallRollAngle, /**< Ball is rolling from a high angle, rotate the robot for better intercepting. */
    (float) bestHitPoint, /**< Best point to intercept ball. */
    (float) footTipYShift, /**< If not much time is left, the ball should be hit with the tip of one if the feet. But with the inner side of the foot. */
    (float) preventSideKickTime, /**< If only this much time is left to intercept the ball, do not try it. */
  }),
});

class InterceptBallProvider : public InterceptBallProviderBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theInterceptBallGenerator The representation updated.
   */
  void update(InterceptBallGenerator& theInterceptBallGenerator) override;

  /**
   * This method calculate how to intercept the ball
   * @param motionRequest The motionRequest
   * @param lastPhase The last motion
   * @param isLeftPhase Next phase is a left one
   * @param translationPolygon The translation polygon used for clipping
   * @param oldStep Last intercept step
   * @return The walk step
   */
  Pose2f intercept(const MotionRequest& motionRequest, const MotionPhase* lastPhase, const bool isLeftPhase, const std::vector<Vector2f>& translationPolygon, const std::optional<Vector2f>& oldStep);

  /**
   * This method clips a provided pose such that the WalkingEngined doesn't complain
   * @param pose Pose to be clipped
   * @param isLeftPhase If the current phase is a left phase
   * @param lastPhase The last motion phase
   * @param isIntercepting Is this intercepting or trying to kick the ball?
   */
  void clipPose(Pose2f& pose, bool isLeftPhase, const MotionPhase& lastPhase, const bool isIntercepting);

  /**
   * This method clips a provided pose such that the WalkingEngined doesn't complain
   * @param pose Pose to be clipped
   * @param isLeftPhase If the current phase is a left phase
   * @param translationPolygon The translationpolygon used for clipping
   */
  void clipPose(Pose2f& pose, bool isLeftPhase, const std::vector<Vector2f>& translationPolygon);

  bool calculatedRectangles = false; /**< Only calculate the rectangles once. */

  // Areas for just intercepting
  Vector2f backRightIntercepting; /**< The back right point of the foot rectangle for intercepting */
  Vector2f frontLeftIntercepting; /**< The from left point of the foot rectangle for intercepting */
};
