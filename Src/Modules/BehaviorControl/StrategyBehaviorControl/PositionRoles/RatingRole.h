/**
 * @file RatingRole.h
 *
 * This file declares the RatingRole a base class for position roles based on maximizing a rating function.
 * It tries to maximize a rating function.
 * For that it uses gradient ascent.
 *
 * @author Yannik Meinken
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/PositionRole.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"

class RatingRole : public PositionRole
{
  STREAMABLE(Parameters,
  {,
    (float) delta, /**< how far the samples for the numerical gradient are from the last position */
    (float) step, /**< step size for gradient ascend */
    (float) cellSize, /**< Size of each grid cell in mm on the field, lower number results in higher resolution for the heatmap */
    (int) numIterations, /**< how many iterations the gradient ascend does each cycle, does not need to be high as we start at the position computed in the previous cycle */
    (unsigned char) heatmapAlpha, /**< Transparency of the heatmap between 0 (invisible) and 255 (opaque) */
    (ColorRGBA) worstEvaluationColor, /**< Red color in RGB corresponding to a pass evaluation value of 0 in the heatmap */
    (ColorRGBA) bestEvaluationColor, /**< Blue color in RGB corresponding to a pass evaluation value of 1 in the heatmap */
    (unsigned) positionBufferSize, /**< The size of the position buffer. */
    (float) baseDeviation, /**< The deviation around the base pose from where to draw sample points for random new initialization */
    (float) lastPosBonus, /**< Hysteresis for last position rating. */

    (float) stopTeammateDistance, /**< Min distance to teammates to allow stopping based on rating alone */

    // start and stop thresholds (can (and some should) be overridden by the explicit role)
    (float) startThreshold, /**< the rating has to be at least this much better at the destination (normalized for the target pose) to start moving. Defaults to zero as the value is heavily dependent on the rating function and should be overridden by the explicit role */
    (float) stopThreshold, /**< if the rating is not at least this much better at the destination (normalized for the target pose) stop moving. Defaults to zero as the value is heavily dependent on the rating function and should be overridden by the explicit role */
    (Angle) startAngle, /**< threshold for how much the angle can differ before correction starts */
    (Angle) stopAngle, /**< threshold for how much the angle can differ to stop non the less */
    (float) startTranslationThreshold, /**< threshold for how much the translation pose can differ before correction starts */
    (float) stopTranslationThreshold, /**< threshold for how much the translation pose can differ to stop non the less */
  });

  Pose2f position(Side side, const Pose2f& basePose, const std::vector<Vector2f>& baseArea, const Agent& self, const Agents& teammates) override;

  //returns true if the current pose is not good enough, so we should adapt
  bool shouldStart(const Pose2f& target) const override;

  //returns true if the current pose is good enough to stop moving
  bool shouldStop(const Pose2f& target) const override;

  void reset() override;

  /**
   * Moves the position along the gradient of the rating function for numIterations steps.
   * @param pos Position from which to start and which will be moved.
   * @param numIterations How many iteration should be done.
   */
  void gradientAscent(Vector2f& pos, const int numIterations) const;

  //computes the rating for a given point
  virtual float rating(const Vector2f& pos) const = 0;

  //draws the rating as a heatmap
  void drawRating();

  bool changedRole = true;
  Vector2f pos = Vector2f::Zero();

  const Vector2f opponentGoal = Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f);

  //needed for drawing
  Vector2f upperFieldCorner = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftTouchline);
  Vector2f gridCornerLower; /**< Corner point with smallest coordinates for the heatmap to be to be drawn */
  Vector2f gridCornerUpper; /**< Corner point with largest coordinates for the heatmap to be drawn */
  Vector2i cellsNumber; /**< Number of grid cells on the corresponding axis, or resolution for the heatmap */
  int totalCellsNumber;
  std::vector<ColorRGBA> cellColors;
  Rangef minMax = {0, 1};

protected:
  Parameters p;

private:
  RingBufferWithSum<Vector2f> positionBuffer{Vector2f::Zero(), p.positionBufferSize};

protected:
  Vector2f base;
  std::vector<Vector2f> region;
  Vector2f ball;
  Agent agent; //The agent which executes the role.

  bool drawHeatmap = false;

  void preProcess() override;

  float getClosestTeammateSquaredDistance(const Vector2f& fieldPos) const;

public:
  RatingRole();
};
