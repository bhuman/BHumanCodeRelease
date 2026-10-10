/**
 * @file DribbleEngine.h
 *
 * This file declares a module that provides a dribble generator.
 *
 * @author Philip Reichenberg
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/DribbleGenerator.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkToBallGenerator.h"
#include "Framework/Module.h"

MODULE(DribbleEngine,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FrameInfo),
  REQUIRES(KickInfo),
  REQUIRES(OdometryDataPreview),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(WalkToBallGenerator),
  PROVIDES(DribbleGenerator),
  DEFINES_PARAMETERS(
  {,
    (float)(100.f) minBallPositionFuture, /**< Ball must land this far away from us if it is rolling towards us. */
    (float)(500.f) minBallPositionFrontSide, /**< Ball must be this far away relative to the closest point it will have to us. */
    (float)(300.f) minBallVelocityCloseRange, /**< Clip ball velocity to this value when close to the ball. */
    (Rangef)(250.f, 2000.f) ballVelocityInterpolationRange, /**< Based on the current ball distance interpolate the velocity. */
  }),
});

class DribbleEngine : public DribbleEngineBase
{
  void update(DribbleGenerator& dribbleGenerator) override;

  /**
   * Calculates the pose where the robot should walk to be able to dribble the ball with one step.
   * @param ballInSCS The ball position relative to the new support foot frame.
   * @param directionInSCS The direction relative to the new support foot frame to which the ball should be dribbled.
   * @return The pose from which the ball can be dribbled with one step.
   */
  Pose2f calcBasePose(const Vector2f& ballInSCS, Angle directionInSCS) const;

  /**
   * Calculate a temp kick pose in case the ball is rolling towards us.
   * We can not trust the perceived ball velocity and direction. Therefore clip the ball position, to avoid turning away from the ball.
   * @param motionRequest Motion request to access original ball velocity
   * @param ballSCS ball position used to determine the kick pose
   * @param ballInSCSNow current used relative ball position
   * @param scsCognition transformation pose to get a relative field position into a zero-step relative position (only rotation is used)
   */
  void calcInterceptionPosition(const MotionRequest& motionRequest, Vector2f& ballSCS,
                                const Vector2f& ballInSCSNow, const Pose2f& scsCognition);
};
