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
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Tools/Module/Module.h"

MODULE(FieldBallProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  PROVIDES(FieldBall),
  DEFINES_PARAMETERS(
  {,
    (float)(300.f) opponentGoalpostYOffset,     /**< Assume that the opponent goal is a bit wider */
    (float)(300.f) ownGoalpostYOffset,          /**< Assume that the own goal is a bit wider */
  }),
});

class FieldBallProvider : public FieldBallProviderBase
{
  /** Computes and fills the elements of the representation
   * @param fieldBall The additional ball information for the behavior
   */
  void update(FieldBall& fieldBall) override;

  /** Predicts ball motion and checks for intersection with own and opponent goal line
   * @param fieldBall The additional ball information for the behavior
   */
  void checkIfBallIsRollingTowardsAGoal(FieldBall& fieldBall);

  /** Predicts ball motion and checks for intersection with the own local y axis
   * @param fieldBall The additional ball information for the behavior
   */
  void checkIfBallIsPassingOwnYAxis(FieldBall& fieldBall);

  /** Checks, if ball position is inside own penalty area
   * @param fieldBall The additional ball information for the behavior
   */
  void checkIfBallIsInsideOwnPenaltyArea(FieldBall& fieldBall);
};
