/**
 * @file Forward.h
 *
 * This file declares the forward role.
 * It tries to maximize a rating function in order to receive a pass and score a goal.
 * For that it uses gradient ascent.
 *
 * @author Yannik Meinken
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/PositionRole.h"

class Forward : public PositionRole
{
  STREAMABLE(Parameters,
  {,
    (float)(100.f) delta, /**< how far the samples for the numerical gradient are from the last position */
    (float)(100.f) step, /**< step size for gradient ascend */
    (float)(2000.f) sigmaBase, /**< standard deviation for rating dependent on base position */
    (float)(50.f) cellSize, /**< Size of each grid cell in mm on the field, lower number results in higher resolution for the heatmap */
    (float)(0.4f) startThreshold, /**< the rating has to be at least this much better at the destination to start moving */
    (float)(0.2f) stopThreshold, /**< if the rating is not at least this much better at the destination stop moving */
    (Angle)(30_deg) startAngle, /**< threshold for how much the angle can differ before correction starts */
    (Angle)(10_deg) stopAngle, /**< threshold for how much the angle can differ to stop non the less */
    (float)(250.f) startTranslationThreshold, /**< threshold for how much the translation pose can differ before correction starts */
    (float)(150.f) stopTranslationThreshold, /**< threshold for how much the translation pose can differ to stop non the less */
    (float)(0.00001f) addWeight, /**< how much should the individual rating functions be added additionally to the multiplication */
    (int)(5) numIterations, /**< how many iterations the gradient ascend does each cycle, does not need to be high as we start at the position computed in the previous cycle */
    (unsigned char)(255) heatmapAlpha, /**< Transparency of the heatmap between 0 (invisible) and 255 (opaque) */
    (ColorRGBA)(213, 17, 48) worstEvaluationColor, /**< Red color in RGB corresponding to a pass evaluation value of 0 in the heatmap */
    (ColorRGBA)(0, 104, 180) bestEvaluationColor, /**< Blue color in RGB corresponding to a pass evaluation value of 1 in the heatmap */
  });

  void preProcess() override;

  Pose2f position(Side side, const Pose2f& basePose, const std::vector<Vector2f>& baseArea, const Agents& teammates) override;

  //returns true if the current pose is not good enough so we should adapt
  bool shouldStart(const Pose2f& target) const override;

  //returns true if the current pose is good enough to stop moving
  bool shouldStop(const Pose2f& target) const override;

  void reset() override;

  /// <summary>
  /// moves the position along the gradient of the rating function for numIterations steps
  /// </summary>
  /// <param name="pos">: position from which to start and which will be moved</param>
  /// <param name="numIterations">: how many iteration should be done</param>
  void gradientAscent(Vector2f& pos, const int numIterations) const;

  //computes the rating for a given point
  float rating(const Vector2f& pos) const;

  //draws the rating as a heatmap
  void drawRating();

  Parameters p;

  bool changedRole = true;
  Vector2f pos = Vector2f::Zero();

  Vector2f base;
  std::vector<Vector2f> region;
  Vector2f ball;

  bool drawHeatmap = false;

  //needed for drawing
  Vector2f upperFieldCorner = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline);
  Vector2f gridCornerLower; /**< Corner point with smallest coordinates for the heatmap to be to be drawn */
  Vector2f gridCornerUpper; /**< Corner point with largest coordinates for the heatmap to be drawn */
  Vector2i cellsNumber; /**< Number of grid cells on the corresponding axis, or resolution for the heatmap */
  int totalCellsNumber;
  std::vector<ColorRGBA> cellColors;
  Rangef minMax = {0, 1};
};
