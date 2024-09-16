/**
 * @file FieldInterceptBallProvider.h
 *
 * Provides information about intercepting the ball for the Behavior
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/FallDownState.h"
#include "Framework/Module.h"

MODULE(FieldInterceptBallProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(BehaviorParameters),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(KickInfo),
  REQUIRES(RobotPose),
  REQUIRES(FieldBall),
  PROVIDES(FieldInterceptBall),
  LOADS_PARAMETERS(
  {,
    (float) ballInterceptionWidthHysteresis, /**< When intercepting, the interception point must be this much more further away. */
    (bool) useXAxisIntersection, /**< Use x axis interception. */
    (bool) useRiskyBallEstimate, /**< Use the risky ball estimate. */
    (Rangef) interpolateRiskyBallEstimateRange, /**< Interpolate between normal and risky ball estimate based on the distance to the ball. */
  }),
});

class FieldInterceptBallProvider : public FieldInterceptBallProviderBase
{
  bool useRiskyBallEstimateAllowed = false; /**< Use the risky ball estimate. */

  /** Computes and fills the elements of the representation
   * @param fieldBall The additional ball information for the behavior
   */
  void update(FieldInterceptBall& theFieldInterceptBall) override;

  /** Predicts ball motion and checks for intersection with the own local y axis
   * @param intersectionPositionWithOwnYAxis The point the ball will intersect with the own relative y-axis
   * @param timeUntilIntersectsOwnYAxis Time until intersection (in s)
   * @param estimate The ball estimate to be used
   */
  void checkIfBallIsPassingOwnYAxis(Vector2f& intersectionPositionWithOwnYAxis, float& timeUntilIntersectsOwnYAxis, const Vector2f& ballPosition, const Vector2f& ballVelocity, const bool distanceCheck);

  void checkIfBallIsPassingOwnXAxis(Vector2f& intersectionPositionWithOwnXAxis, float& timeUntilIntersectsOwnXAxis, const Vector2f& ballPosition, const Vector2f& ballVelocity, const bool distanceCheck);

  /** Calculates the position where the ball can be intercepted
   * @param fieldBall The additional ball information for the behavior
   */
  void calculateInterceptedBallEndPosition(FieldInterceptBall& theFieldInterceptBall);
};
