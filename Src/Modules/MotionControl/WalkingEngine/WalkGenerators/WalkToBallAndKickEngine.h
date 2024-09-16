/**
 * @file WalkToBallAndKickEngine.h
 *
 * This file declares a module that provides a walk to ball and kick generator.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/MotionControl/KickGenerator.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Representations/MotionControl/WalkToBallAndKickGenerator.h"
#include "Representations/MotionControl/WalkToBallGenerator.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Framework/Module.h"

MODULE(WalkToBallAndKickEngine,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FrameInfo),
  REQUIRES(KickGenerator),
  REQUIRES(KickInfo),
  USES(MotionInfo),
  REQUIRES(MotionRequest),
  REQUIRES(ObstacleModel),
  REQUIRES(OdometryDataPreview),
  REQUIRES(OdometryTranslationRequest),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkGenerator),
  REQUIRES(WalkKickGenerator),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(WalkToBallGenerator),
  PROVIDES(WalkToBallAndKickGenerator),
  DEFINES_PARAMETERS(
  {,
    (Rangef)(Rangef(35.f, 65.f)) forwardFastYThreshold, /**< Relative y ball position to kick foot. */
    (Rangef)(Rangef(100.f, 215.f)) forwardFastXThreshold, /**< Relative x ball position to kick foot. */
    (float)(240.f) forwardFastLongXMaxThreshold,  /**< Relative max x ball position to kick foot. */
    (Pose2f)(Pose2f(20_deg, 35.f, 10.f)) forwardStealStepPlanningThreshold, /**< Max needed step size for reach position for forwardSteal, to start V-Shape position. */
    (float)(15.f) forwardStealXBallUnseenThreshold, /**< Shift forward steal pose by this amount if ball is not seen. */
    (Rangef)(Rangef(50.f, 350.f)) forwardStealBallUnseenInterpolationRange,
    (float)(50.f) kickPoseMaxYTranslation, /**< Kick pose y-Translation must be smaller than this threshold. */
    (Rangef)(Rangef(-80.f, -40.f)) forwardFastYClipRange, /**< For the dynamic points, clip the y position. */
    (Pose2f)(Pose2f(7_deg, 30.f, 40.f)) robotStuckThresholds, /**< Thresholds to start the V-shape kickpose. */
    (float)(100.f) minBallPositionFuture, /**< Ball must land this far behind us if it is rolling towards us. */
    (float)(500.f) minBallPositionFrontSide, /**< Ball must be this far away relative to the closest point it will have to us. */
    (float)(50.f) minBallVelocityCloseRange, /**< Clip ball velocity to this value when close to the ball. */
    (Rangef)(150.f, 2000.f) ballVelocityInterpolationRange, /**< Based on the current ball distance interpolate the velocity. */
    (float)(50.f) maxBallVelocityKickEngine, /**< Max allowed ball velocity for KickEngine kicks. */
    (float)(500.f) maxBallVelocityInWalkKick, /**< Max allowed ball velocity for WalkKickEngine kicks. */
  }),
});

class WalkToBallAndKickEngine : public WalkToBallAndKickEngineBase
{
  void update(WalkToBallAndKickGenerator& walkToBallAndKickGenerator) override;
  bool lastPhaseWasKick = false; /**< Was last motion phase a kick phase? */
  bool lastPhaseWasKickPossible = false; /**< Last phase the kick was possible. */
  bool ignoreBallTimestamp = false;
  Vector2f lastStableBall = Vector2f(0.f, 0.f);

  OdometryDataPreview lastOdometry;
  OdometryTranslationRequest lastOdometryRequest;

  /**
   * Creates the kick phase for the given request.
   * @param motionRequest The name says it all.
   * @param lastPhase The name says it all.
   * @return
   */
  std::unique_ptr<MotionPhase> createKickPhase(const MotionRequest& motionRequest, const MotionPhase& lastPhase,
                                               const WalkKickVariant& walkKickDirection);

  /**
   * Shift the kick pose away from the obstacle for the forward and turn kicks
   */
  float shiftInWalkKickInYDirection(const Pose2f& kickPose, const KickInfo::KickType kickType, const Angle direction);

  /**
   * Calculate a temp kick pose in case the ball is rolling towards us
   */
  bool calcInterceptionPosition(const MotionRequest& motionRequest, Vector2f& ballSCS,
                                const Vector2f& ballInSCSNow, const Pose2f& scsCognition);
};
