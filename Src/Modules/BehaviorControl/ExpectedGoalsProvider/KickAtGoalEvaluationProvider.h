/**
 * @file KickAtGoalEvaluationProvider.h
 *
 * This file declares a module that evaluates target positions for passes.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Math/BHMath.h"
#include "Representations/BehaviorControl/ExpectedGoals.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/KickAtGoalEvaluation.h"
#include "Representations/BehaviorControl/PassEvaluation.h"
#include "Representations/Communication/SentTeamMessage.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include <map>

MODULE(KickAtGoalEvaluationProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldBall),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(KickInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  PROVIDES(KickAtGoalEvaluation),
  LOADS_PARAMETERS(
  {,
    (float) hysteresisNumber, /**< A number which is used in various places to define a hysteresis offset. */
    (float) hysteresisKickRangeExtension, /**< The range of the previously selected kick is extended by this length. */
    (Angle) hysteresisAngle, /**< When sorting sectors by their opening angle, the one selected in the previous frame gets this as bonus. */
    (Angle) minOpeningAngle, /**< The minimum opening angle a sector must have to be considered. */
    (Angle) kickInaccuracy, /**< The search angle range is narrowed by this to both sides. */
    (float) sectorShift, /**< Shift direction by this factor times sector width towards center. */
    (KickInfo::KickType) useKick,
    (KickInfo::KickType) replaceKick, /**< If an obstacle is too close, replace the kick with a softer one. */
    (Rangea) distanceToObstacle, /**< Near own goal, replace strong kick with softer one when an obstacle is too close. */
    (bool) ignoreRangeLimit, /**< Ignore that the kick can not reach the goal according to theKickInfo. */
  }),
});

class KickAtGoalEvaluationProvider : public KickAtGoalEvaluationProviderBase
{
public:
  KickAtGoalEvaluationProvider();
  void update(KickAtGoalEvaluation& theKickAtGoalEvaluation) override;

private:

  struct ObstacleSector
  {
    Rangea sector; /**< The angular range relative to the ball that the obstacle blocks. */
    float distance; /**< The distance of the obstacle to the ball. */
    float x; /**< The x coordinate on field of the obstacle. */
  };

  bool wasValidLastFrame = false;
  bool wasUpdatedLastFrame = false; /**< Was the module called last frame? */
  bool aimingAtGoal = false; /**< Whether the robot is able to score a goal in the current state. */
  Vector2f goalLineIntersection = Vector2f::Zero(); /**< The point where the ball would have intersected the goal line according to the current plan. */

  const Vector2f leftGoalPost; /**< The position of the left post of the opponent's goal. */
  const Vector2f rightGoalPost; /**< The position of the right post of the opponent's goal. */
  const Geometry::Line goalLine; /**< The line which the ball has to cross to be inside the goal. */

  Angle getDistanceToNearestObstacle(const KickAtGoalEvaluation& theKickAtGoalEvaluation, const std::list<SectorWheel::Sector>& sectors);
};
