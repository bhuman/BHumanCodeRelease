/**
 * @file InterceptBallProviderDBHC.h
 *
 * This file declares a module that intercepts the ball.
 * This version was ONLY used in the DBHC.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/InterceptBallGenerator.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Representations/MotionControl/WalkToPoseGenerator.h"
#include "Representations/MotionControl/WalkAtAbsoluteSpeedGenerator.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(InterceptBallProviderDBHC,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(KickInfo),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkAtAbsoluteSpeedGenerator),
  REQUIRES(WalkGenerator),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(WalkKickGenerator),
  REQUIRES(WalkToPoseGenerator),
  PROVIDES(InterceptBallGenerator),

  DEFINES_PARAMETERS(
  {,
    (Rangef)(-40.f, 40.f) maxYShift,
    (Rangef)(-20.f, 10.f) maxXStep,
    (Angle)(40_deg) maxRotationStep,
    (Angle)(150_deg) maxBallRollAngle,
    (float)(2.f) minTimeForBallAlwaysRotate,
    (float)(1000.f) minBallDistanceAlwaysRotate,
    (float)(100.f) bestHitPoint,
  }),
});

class InterceptBallProviderDBHC : public InterceptBallProviderDBHCBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theInterceptBallGenerator The representation updated.
   */
  void update(InterceptBallGenerator& theInterceptBallGenerator) override;

  float yLegIntercept();

  float distanceBallEndToFieldBorderSquared(const Pose2f& scsCognition, const Vector2f& ballPosition, const Vector2f& ballEndPosition, const Vector2f& ballVelocity);

  float distanceBallToFieldBorderSideKick(const Pose2f& scsCognition, const std::vector<Angle>& directions);

  std::optional<Vector2f> fieldBorderBottomRight;
  std::optional<Vector2f> fieldBorderTopLeft;
};
