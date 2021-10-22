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
#include "Tools/Module/Module.h"

MODULE(WalkToBallAndKickEngine,
{,
  REQUIRES(BallSpecification),
  REQUIRES(GroundContactState),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(KickGenerator),
  REQUIRES(KickInfo),
  REQUIRES(MassCalibration),
  USES(MotionInfo),
  REQUIRES(MotionRequest),
  REQUIRES(ObstacleModel),
  USES(OdometryData),
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
    (Rangef)(Rangef(45.f, 60.f)) forwardFastYThreshold, /**< Relativ y ball position to kick foot. */
    (Rangef)(Rangef(100.f, 215.f)) forwardFastXThreshold, /**< Relativ x ball position to kick foot. */
    (float)(240.f) forwardFastLongXMaxThreshold,  /**< Relativ max x ball position to kick foot. */
    (float)(50.f) maxBallVelocity,  /**< Max allowed ball velocity. TODO: or 150.f for some kicks? */
    (Pose2f)(Pose2f(20_deg, 20.f, 10.f)) forwardStealStepPlanningThreshold, /**< Max needed step size for reach position for forwardSteal, to start V-Shape position. */
    (float)(100.f) kickPoseMaxYTranslation, /**< Kick pose y-Translation must be smaller than this threshold. */
    (Angle)(30_deg) forwardStealVFeetAngle, /**< Requested V-Shape of the feet. */
    (Rangef)(Rangef(-80.f, -40.f)) forwardFastYClipRange, /**< For the dynamic points, clip the y position. */
    (float)(0.8f) minBallDistanceForVelocity, /**< Subtract this much time to reach the ball, when propagating the ball position. */
    (Pose2f)(Pose2f(4_deg, 10.f, 10.f)) kickPoseThresholds,
  }),
});

class WalkToBallAndKickEngine : public WalkToBallAndKickEngineBase
{
  void update(WalkToBallAndKickGenerator& walkToBallAndKickGenerator) override;
  float overrideKickPower = -1.f;
  bool lastPhaseWasKick = false;
  bool lastPhaseWasKickPossible = false;
  Vector2f lastStableBall = Vector2f(0.f, 0.f);
  /**
   * Creates the kick phase for the given request.
   * @param motionRequest The name says it all.
   * @param lastPhase The name says it all.
   * @return
   */
  std::unique_ptr<MotionPhase> createKickPhase(const MotionRequest& motionRequest, const MotionPhase& lastPhase, const Angle targetDirection,
                                               const bool mirrorKick, const Rangea& precisionRange, const float kickPoseShiftY);

  float shiftInWalkKickInYDirection(const Pose2f& kickPose, const KickInfo::KickType kickType, const Angle direction);
};
