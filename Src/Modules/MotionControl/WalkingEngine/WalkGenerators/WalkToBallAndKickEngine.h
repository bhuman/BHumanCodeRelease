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
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkToBallAndKickGenerator.h"
#include "Representations/MotionControl/WalkToBallGenerator.h"
#include "Framework/Module.h"

MODULE(WalkToBallAndKickEngine,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FrameInfo),
  REQUIRES(KickInfo),
  REQUIRES(OdometryDataPreview),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(WalkToBallGenerator),
  PROVIDES(WalkToBallAndKickGenerator),
  DEFINES_PARAMETERS(
  {,
    (float)(100.f) minBallPositionFuture, /**< Ball must land this far behind us if it is rolling towards us. */
    (float)(500.f) minBallPositionFrontSide, /**< Ball must be this far away relative to the closest point it will have to us. */
    (float)(50.f) minBallVelocityCloseRange, /**< Clip ball velocity to this value when close to the ball. */
    (Rangef)(150.f, 2000.f) ballVelocityInterpolationRange, /**< Based on the current ball distance interpolate the velocity. */
  }),
});

class WalkToBallAndKickEngine : public WalkToBallAndKickEngineBase
{
  void update(WalkToBallAndKickGenerator& walkToBallAndKickGenerator) override;

  /**
   * Calculate a temp kick pose in case the ball is rolling towards us
   */
  bool calcInterceptionPosition(const MotionRequest& motionRequest, Vector2f& ballSCS,
                                const Vector2f& ballInSCSNow, const Pose2f& scsCognition);
};
