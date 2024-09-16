/**
 * @file ExpectedGoalsProvider.h
 *
 * This file declares a module that estimates the probability of scoring a goal when shooting from a hypothetical ball position.
 *
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/BehaviorControl/ExpectedGoals.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Tools/BehaviorControl/SectorWheel.h"

MODULE(ExpectedGoalsProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FieldDimensions),
  REQUIRES(GlobalOpponentsModel),
  PROVIDES(ExpectedGoals),
  DEFINES_PARAMETERS(
  {,
    (float)(0.01f) minValue, /**< Minimum probability of a successful goal shot in the worst situation possible. */
    (Angle)(0_deg) minOpeningAngle, /**< Opening angle for the goal shot line to be considered blocked */
    (Angle)(20_deg) maxOpeningAngle, /**< Opening angle for the goal shot line to be considered free */
    (Angle)(40_deg) maxOpeningAngleOpponenent, /**< Opening angle for the goal shot line to be considered free */
    (float)(100.f) cellSize, /**< Size of each grid cell in mm on the field, lower number results in higher resolution for the heatmap */
    (unsigned char)(255) heatmapAlpha, /**< Transparency of the heatmap between 0 (invisible) and 255 (opaque) */
    (ColorRGBA)(213, 17, 48) worstRatingColor, /**< Red color in RGB corresponding to a pass rating value of 0 in the heatmap */
    (ColorRGBA)(0, 104, 180) bestRatingColor, /**< Blue color in RGB corresponding to a pass rating value of 1 in the heatmap */
  }),
});

class ExpectedGoalsProvider : public ExpectedGoalsProviderBase
{
public:
  void update(ExpectedGoals& theExpectedGoals) override;
  ExpectedGoalsProvider();

private:
  bool drawHeatmap = false;
  bool calcOpeningAngle = true;
  bool calcShotDistance = true;
  bool isPositioningDrawing = false;
  Vector2i cellsNumber; /**< Resolution for the heatmap i.e. number of grid cells on the corresponding axis */
  std::vector<ColorRGBA> cellColors;

  const Vector2f goalCenter = Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f);
  const Vector2f leftGoalPost = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
  const Vector2f rightGoalPost = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
  const Vector2f gridCornerUpper = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftTouchline);
  const Vector2f gridCornerLower = Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightTouchline);

  /**
   * Estimates the probability of scoring a goal when shooting from a given position, not taking into account the known obstacles.
   * @param pointOnField The position to shoot from.
   * @return The estimated probability of scoring a goal.
   */
  float xG(const Vector2f& pointOnField) const;

  /**
   * Estimates the probability of an opponent missing the goal when shooting from a given position, not taking into account the known obstacles.
   * @param pointOnField The position to shoot from.
   * @return The estimated probability of missing the goal.
   */
  float xGA(const Vector2f& pointOnField) const;

  /**
   * Estimates the probability that a given position has a wide enough opening angle on the opponent's goal to score a goal, taking into account the known obstacles.
   * @param pointOnField The position to shoot from.
   * @param isPositioning Is the rating for the positioning role?
   * @return The estimated probability of scoring a goal.
   */
  float getRating(const Vector2f& pointOnField, const bool isPositioning) const;

  /**
   * Estimates the probability that a given position does not have a wide enough opening angle on the own goal for an opponent to score a goal against, not taking into account the known obstacles.
   * @param pointOnField The position to shoot from.
   * @return The estimated probability of missing the goal.
   */
  float getOpponentRating(const Vector2f& pointOnField) const;

  /**
   * Calculates the opening angle on the opponent's goal from the given position, taking into account the known obstacles.
   * @param pointOnField The position to shoot from.
   * @param isPositioning Is the rating for the positioning role?
   * @return The opening angle on the opponent's goal.
   */
  Angle getOpeningAngle(const Vector2f& pointOnField, const bool isPositioning) const;

  /**
   * Calculates the opening angle on the own goal from the given position, not taking into account the known obstacles.
   * @param pointOnField The position to shoot from.
   * @return The opening angle on the own goal.
   */
  Angle getOpponentOpeningAngle(const Vector2f& pointOnField) const;

  void draw();
};
