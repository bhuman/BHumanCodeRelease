/**
 * @file InterceptBallProvider.h
 *
 * This file implements a module that intercepts the ball
 *
 * @author Florian Scholz
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/InterceptBallGenerator.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(InterceptBallProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FieldInterceptBall),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(KickInfo),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkGenerator),
  REQUIRES(WalkKickGenerator),
  REQUIRES(WalkStepData),
  PROVIDES(InterceptBallGenerator),

  LOADS_PARAMETERS(
  {,
    (bool) kickBall, /**< Allow kicking the rolling ball. */
    (float) timeOffset, /**< Time for start of a kick and touching the ball. */
    (Angle) rotationThreshold, /**< The kickPose must have less than this rotation to allow for a kick. */
    (float) footRectLengthOffset, /**< Length from tip of toe to a forward point, which can realistically reach the ball. Used to check if a kick is possible. */
    (float) footRectYOffset, /**< Length from origin of foot to the side. Used to check if a kick is possible. */
    (float) footRectXSmallOffset, /**< Offset for footRectLengthOffset. This area is used to optimize the kickPose. */
    (float) footRectYSmallOffset, /**< Length from origin of foot to the side, to optimize the kickPose. */
    (float) timeToKickThresholdWalk, /**< The time until the ball reaches the optimal point must be lower than this value to start the kick when walking. **/
    (float) timeToKickThresholdStand, /**< The time until the ball reaches the optimal point must be lower than this value to start the kick when standing. **/
    (float) timeToKickThresholdMirrorBonus, /**< The mirror kick gets a bonus. */
    (Rangef) kickPoseSizeScaling, /**< The closer the ball, the less we want to move. */
    (Rangef) maxYShift, /**< Optimize interception point. */
    (Rangef) maxXStep, /**< Optimize interception point. */
    (Angle) maxRotationStep, /**< For intercepting, use this rotation threshold. */
    (Angle) maxBallRollAngle, /**< Ball is rolling from a high angle, rotate the robot for better intercepting. */
    (float) bestHitPoint, /**< Best point to intercept ball. */
    (float) footTipYShift, /**< If not much time is left, the ball should be hit with the tip of one if the feet. But with the inner side of the foot. */
    (float) preventSideKickTime, /**< If only this much time is left to intercept the ball, do not try it. */
    (Rangef) keepStandingThreshold, /**< If only this much x-translation is requested, we keep standing. */
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
   * This method is used to check the requested KickType
   * @param kickType The requested kick type
   * @return Whether the requested kick type is allowed
   */
  bool checkKickType(const KickInfo::KickType kickType);

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
   * @param fastWalk Use larger minimal x-translation steps
   * @param maxPossibleStepSize Use large side steps
   */
  void clipPose(Pose2f& pose, bool isLeftPhase, const MotionPhase& lastPhase, const bool fastWalk, const bool maxPossibleStepSize);

  /**
   * This method clips a provided pose such that the WalkingEngined doesn't complain
   * @param pose Pose to be clipped
   * @param isLeftPhase If the current phase is a left phase
   * @param translationPolygon The translation polygon used for clipping
   */
  void clipPose(Pose2f& pose, bool isLeftPhase, const std::vector<Vector2f>& translationPolygon);

  KickInfo::KickType getDribbleKick(const MotionRequest& motionRequest, const bool isLeftPhase);

  /**
   * This method calculates the kickPose for a given kick.
   * It also checks if the mirrored kick would be faster to execute.
   * @param kickType The kick
   * @param kickPose The kickPose, that will be returned
   * @param orthPoint The orthPoint of the optimized kick
   * @param ballLine The ball position and movement direction
   * @param scsCognition The transformation from cognition to motion
   * @param motionRequest The motion request
   */
  void optimizeKick(const KickInfo::KickType& kickType, Pose2f& kickPose, Vector2f& orthPoint,
                    const Geometry::Line& ballLine, const Pose2f& scsCognition,
                    const MotionRequest& motionRequest);

  bool planCurrentKick(WalkKickVariant& walkKickVariant, Pose2f& kickPose, float& timeForDistanceHitPoint,
                       const KickInfo::KickType& kickType, const Geometry::Line& ballLine, const Pose2f& scsCognition,
                       const MotionRequest& motionRequest, const bool isLeftPhase, const MotionPhase* lastPhase,
                       const std::vector<Vector2f>& translationPolygon);

  bool calculatedRectangles = false; /**< Only calculate the rectangles once. */

  // Smaller areas for kickPose
  Vector2f leftFootBackRightSmall; /**< The back right point of the left foot rectangle */
  Vector2f leftFootFrontLeftSmall; /**< The from left point of the left foot rectangle */
  Geometry::Rect leftFootRectSmall; /**< The rectangle in front of the left foot */

  Vector2f rightFootBackRightSmall; /**< The right back point of the right foot rectangle */
  Vector2f rightFootFrontLeftSmall; /**< The from left point of the right foot rectangle */
  Geometry::Rect rightFootRectSmall; /**< The rectangle in front of the right foot */

  // Bigger areas for walk kick
  Vector2f leftFootBackRight; /**< The back right point of the left foot rectangle */
  Vector2f leftFootFrontLeft; /**< The from left point of the left foot rectangle */
  Vector2f rightFootBackRight; /**< The right back point of the right foot rectangle */
  Vector2f rightFootFrontLeft; /**< The from left point of the right foot rectangle */

  // Areas for just intercepting
  Vector2f backRightIntercepting; /**< The back right point of the foot rectangle for intercepting */
  Vector2f frontLeftIntercepting; /**< The from left point of the foot rectangle for intercepting */
};
