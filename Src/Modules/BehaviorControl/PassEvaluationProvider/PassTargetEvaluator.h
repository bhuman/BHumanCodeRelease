/**
 * @file PassTargetEvaluator.h
 *
 * This file declares a module that evaluates target positions for passes.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Math/BHMath.h"
#include "Representations/BehaviorControl/ExpectedGoals.h"
#include "Representations/BehaviorControl/PassEvaluation.h"
#include "Representations/BehaviorControl/PassTargetEvaluation.h"
#include "Representations/Communication/SentTeamMessage.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include <map>

MODULE(PassTargetEvaluator,
{,
  REQUIRES(BallSpecification),
  REQUIRES(ExpectedGoals),
  REQUIRES(FieldDimensions),
  REQUIRES(GlobalOpponentsModel),
  REQUIRES(GlobalTeammatesModel),
  REQUIRES(GameState),
  REQUIRES(PassEvaluation),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotPose),
  REQUIRES(SentTeamMessage),
  PROVIDES(PassTargetEvaluation),
  LOADS_PARAMETERS(
  {,
    (Angle) minAngleOffset, /**< Kick angle must deviate more than this from the next obstacle range in the sector wheel. This is half the minimum angular range of a free sector that the ball can be kicked through */
    (Angle) maxPrecisionOffset, /**< Precision range for kick angle may not be more than +- this offset */
    (float) keepOldTargetDistance, /**< Keep the old communicated target if it is close enough to our last calculated one. */
    (Angle) keepOldTargetBearing, /**< Keep the old communicated target if the kick direction is similar. */
    (float) passAheadDistance, /**< Shift the pass target by this distance in the direction of the goal */
    (Angle) maxAngleOffset, /**< Kick angle must not deviate more than this from the original angle to the target. This is half the angular range in which free sectors are considered */
    (Angle) maxAngleOffsetBehind, /**< maxAngleOffset when the ball is passed on the own goal side (instead of the opponent's goal side) of the pass target (from the perspective of the ball) */
    (float) opponentExtraWidth, /**< Extra width for other robots. */
  }),
});

class PassTargetEvaluator : public PassTargetEvaluatorBase
{
public:
  PassTargetEvaluator();
  void update(PassTargetEvaluation& thePassTargetEvaluation) override;

private:

  TargetEvaluation lastTarget;

  bool passHysteresis = false;
  bool resetLastTarget = true;
  Pose2f lastRobotPose;

  TargetEvaluation& getRating(const Vector2f& baseOnField, const Vector2f& targetOnField, const int teammate);
  void calculateTeammatePassTarget(TargetEvaluation& target) const;

  // Calculates a kick target as close to the teammate's position as possible while reducing the likelihood of obstacles blocking the ball on the way. Requires the teammate pointer to be non-null.
  bool findNextFreeAngle(const std::list<SectorWheel::Sector>& sectors, Angle& targetAngle, Rangea& precisionRange, const bool ccw, const Angle maxAngleDeviation, const Angle& modifiedMinAngleOffset) const;

  // Sets the closest target angle and precision range by reference, if a free angle (i.e. not blocked by obstacles) could be found in the sector wheel with the given parameters.
  Angle getNextAngleInRange(const Rangea angleRange, const bool ccw, const Angle& modifiedMinAngleOffset) const;
  std::list<SectorWheel::Sector> calculateObstacleSectors(const Vector2f& ballPositionOnField, const bool isAngleFree, const int teammate) const;
  std::pair<float, float> getAngleRating(const Vector2f& ballPosition, const Angle candidateAngle, const bool isIdealSide, const Vector2f basePosition, const Angle targetAngle, const float targetDistance) const;
  bool setAnglePrecisionRange(Angle& targetAngle, Rangea& precisionRange, const Angle currentAngle, const Angle maxAngleDeviation, const Rangea& angleRange) const;
  std::vector<TargetEvaluation> targetEvaluations;

  const Vector2f opponentGoal;
};
