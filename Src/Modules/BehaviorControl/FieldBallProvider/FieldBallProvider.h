/**
 * @file BehaviorControl/FieldBallProvider/FieldBallProvider.h
 *
 * Provides the FieldBall for the Behavior
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeammatesBallModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Framework/Module.h"

MODULE(FieldBallProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(BehaviorParameters),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(KickInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeammatesBallModel),
  PROVIDES(FieldBall),
  LOADS_PARAMETERS(
  {,
    (float) opponentGoalpostYOffset,     /**< Assume that the opponent goal is a bit wider */
    (float) ownGoalpostYOffset,          /**< Assume that the own goal is a bit wider */
    (float) ballInterceptionWidthHysteresis, /**< When intercepting, the interception point must be this much more further away. */
    (float) consistencyDistanceToDropInPosition,  /**< If the ball is within this distance to a possible ball drop in position for the current game state
                                                            * it is considered to be consistent with the current game state. */
    (bool) useRiskyBallEstimate, /**< Use the risky ball estimate. */
    (Rangef) interpolateRiskyBallEstimateRange, /**< Interpolate between normal and risky ball estimate based on the distance to the ball. */
    (bool) useXAxisIntersection, /**<Use x axis interception. */
  }),
});

class FieldBallProvider : public FieldBallProviderBase
{
  /** Computes and fills the elements of the representation
   * @param fieldBall The additional ball information for the behavior
   */
  void update(FieldBall& fieldBall) override;

  /** Predicts ball motion and checks for intersection with own and opponent goal line
   * @param isRollingTowardsOpponentGoal Is the ball rolling towards the opponent goal?
   * @param isRollingTowardsOwnGoal Is the ball rolling towards the own goal?
   * @param positionOnField Ball position in field coordinates
   * @param velocity Ball velocity (relative)
   */
  void checkIfBallIsRollingTowardsAGoal(bool& isRollingTowardsOpponentGoal, bool& isRollingTowardsOwnGoal,
                                        const Vector2f& positionOnField, const Vector2f& velocity);

  /** Predicts ball motion and checks for intersection with the own local y axis
   * @param intersectionPositionWithOwnYAxis The point the ball will intersect with the own relative y-axis
   * @param timeUntilIntersectsOwnYAxis Time until intersection (in s)
   * @param estimate The ball estimate to be used
   */
  void checkIfBallIsPassingOwnYAxis(Vector2f& intersectionPositionWithOwnYAxis, float& timeUntilIntersectsOwnYAxis, const BallState& estimate);

  void checkIfBallIsPassingOwnXAxis(Vector2f& intersectionPositionWithOwnXAxis, float& timeUntilIntersectsOwnXAxis, const BallState& estimate);

  /** Checks, if ball position is inside own penalty area
   * @param fieldBall The additional ball information for the behavior
   */
  void checkIfBallIsInsideOwnPenaltyArea(FieldBall& fieldBall);

  /** Calculates the position where the ball can be intercepted
   * @param fieldBall The additional ball information for the behavior
   */
  void calculateInterceptedBallEndPosition(FieldBall& fieldBall);

  /**
   * Checks, if the ball position is consistent with the game state.
   * @param fieldBall The additional ball information for the behavior
   * @param theGameState The game state
   */
  void checkBallPositionIsConsistentWithGameState(FieldBall& fieldBall, const GameState& theGameState);
};
