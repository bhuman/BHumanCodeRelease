/**
 * @file PassEvaluationProvider.h
 *
 * This file declares a module that evaluates a target position by estimating how likely it is that a pass of the ball would be successful.
 *
 * @author Jo Lienhoop
 */

#pragma once

#include "Framework/Module.h"
#include "Math/BHMath.h"
#include "Representations/BehaviorControl/ExpectedGoals.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/PassEvaluation.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Infrastructure/GameState.h"

MODULE(PassEvaluationProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(ExpectedGoals),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GlobalOpponentsModel),
  REQUIRES(GameState),
  PROVIDES(PassEvaluation),
  DEFINES_PARAMETERS(
  {,
    (float)(0.01f) minValue, /**< Minimum probability of a successful pass in the worst situation possible. */
    (float)(1400.f) opponentDistToTargetThreshold, /**< Distance to the closest opponent for the pass target to be considered free */
    (float)(1000.f) opponentDistToLineThreshold, /**< Distance to the closest opponent for the pass line to be considered free */
    (float)(4000.f) distancePenaltyThresholdPlaying, /**< if the point is further away than this the distance rating is below 1 */
    (float)(6000.f) distancePenaltyThresholdFreeKick, /**< if the point is further away than this the distance rating is below 1 */
    (float)(1000.f) distancePenaltyDeviation, /**< standard deviation for the upper limit of the distance */
    (float)(800.f) distToBoundaryThreshold, /**< Distance to the closest field boundary for the ball to stay within the field */
    (float)(200.f) opponentShiftToBall, /**< Shift each opponent's position this far in the direction of the ball, assuming their orientation */
    (float)(100.f) obstacleBlockingRadius, /**< Radius of opponents in which a pass would definitely fail */
    (float)(100.f) cellSize, /**< Size of each grid cell in mm on the field, lower number results in higher resolution for the heatmap */
    (Vector2f)(0.f, 500.f) goalPostShift, /**< Shift the blocking area of a teammate's goal shot on the y axis away from the goal posts */
    (unsigned char)(255) heatmapAlpha, /**< Transparency of the heatmap between 0 (invisible) and 255 (opaque) */
    (ColorRGBA)(213, 17, 48) worstRatingColor, /**< Red color in RGB corresponding to a pass rating value of 0 in the heatmap */
    (ColorRGBA)(0, 104, 180) bestRatingColor, /**< Blue color in RGB corresponding to a pass rating value of 1 in the heatmap */
  }),
});

class PassEvaluationProvider : public PassEvaluationProviderBase
{
public:
  void update(PassEvaluation& thePassEvaluation) override;
  PassEvaluationProvider();

private:
  bool drawHeatmap = false;
  bool drawCombinedHeatmap = false;
  bool calcPassTargetFree = true;
  bool calcPassLineFree = true;
  bool calcPassTargetInField = true;
  bool calcShotLineFree = true;
  bool calcShotDistance = true;
  bool isPositioningDrawing = false;
  unsigned int lastUpdateParameters;
  Vector2f leftGoalPost = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal - theBallSpecification.radius * 2.f);
  Vector2f rightGoalPost = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal + theBallSpecification.radius * 2.f);
  Vector2f opponentFieldCorner = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftTouchline);
  Vector2f ownFieldCorner = Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightTouchline);
  std::vector<Vector2f> opponentsOnField;
  Vector2i cellsNumber; /**< Resolution for the heatmap i.e. number of grid cells on the corresponding axis */
  std::vector<ColorRGBA> cellColors;

  /**
   * Estimates the probability that a pass from a given start position (e.g. the current ball position) to a given target position would be successful, taking into account the known obstacles.
   * @param baseOnField The start position to pass from.
   * @param pointOnField The target position to pass to.
   * @param Is the rating for the positioning role?
   * @return The estimated probability of a successful pass.
   */
  float getRating(const Vector2f& baseOnField, const Vector2f& targetOnField, const bool isPositioning);

  /**
   * Updates the member variables related to the ball and obstacles only once per frame.
   */
  void updateParameters();

  /**
   * Updates minimum distance of the obstacles to the target itself as well as the line from the ball to the target.
   * @param baseOnField The start position to pass from.
   * @param targetOnField The target position to pass to.
   * @param obstacleOnField The obstacle to check the distance for.
   * @param minDistToTarget The distance of the closest obstacle to the target position (could be updated by reference).
   * @param minDistToLine The distance of the closest obstacle to the line from the ball to the target position (could be updated by reference).
   */
  void updateMinDistances(const Vector2f& baseOnField, const Vector2f& targetOnField, const Vector2f& obstacleOnField, float& minDistToTarget, float& minDistToLine) const;

  /**
   * Estimates the probability that the position is not blocking a teammate's direct shot at the opponent's goal.
   * @param baseOnField The start position to pass from.
   * @param targetOnField The target position to pass to.
   * @return Probability that the point is outside of the angular range of the ball to the goal posts.
   */
  float isShotLineFree(const Vector2f& baseOnField, const Vector2f& targetOnField) const;

  /**
   * Estimates the probability that the distance from the ball to the target is within the range of a kick.
   * @param baseOnField The start position to pass from.
   * @param targetOnField The target position to pass to.
   * @return Probability that there is a suitable kick.
   */
  float isTargetInRange(const Vector2f& baseOnField, const Vector2f& targetOnField) const;

  /**
   * Calculates distance from a given point to the closest field border.
   * @param pointOnField The target position to pass to.
   * @return Distance to the field border.
   */
  float getDistanceToFieldBorder(const Vector2f& pointOnField) const;

  /**
   * Draws a heatmap on the field with interpolated colors from the rating of the position at each cell of the grid.
   */
  void draw();
};
