/**
 * @file Forward.cpp
 *
 * This file implements the forward role.
 * It tries to maximize a rating function in order to receive a pass and score a goal.
 * For that it uses gradient ascent.
 *
 * @author Yannik Meinken
 */

#include "Forward.h"
#include "Math/Geometry.h"

void Forward::preProcess()
{
  DECLARE_DEBUG_DRAWING("behavior:Forward:position", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:Forward:ascent", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:Forward:heatmap", "drawingOnField");
  MODIFY("parameters:behavior:Forward", p);
  MODIFY_ONCE("behavior:Forward:drawHeatmap", drawHeatmap);
}

Pose2f Forward::position(Side, const Pose2f& basePose, const std::vector<Vector2f>& baseArea, const Agents&)
{
  //initialize parameters needs to be done before every thing else!
  if(changedRole)
    pos = theRobotPose.translation;
  changedRole = false;
  base = basePose.translation;
  region = baseArea;
  ball = theFieldBall.recentBallPositionOnField();

  if(drawHeatmap)
    drawRating();

  pos = rating(pos) > rating(theRobotPose.translation) ? pos : theRobotPose.translation;
  gradientAscent(pos, p.numIterations);

  CROSS("behavior:Forward:position", pos.x(), pos.y(), 100, 20, Drawings::solidPen, ColorRGBA::blue);

  return Pose2f((ball - pos).angle(), pos);
}

bool Forward::shouldStart(const Pose2f& target) const
{
  // TODO: Remove unnecessary brackets
  //TODO: translation and rating should not be combined by disjunction but instead by conjugation
  if((rating(target.translation) - rating(theRobotPose.translation) > p.startThreshold ||
      (std::abs(Angle::normalize(theRobotPose.rotation - (ball - theRobotPose.translation).angle())) > p.startAngle ||
       std::abs(target.translation.x() - theRobotPose.translation.x()) > p.startTranslationThreshold ||
       std::abs(target.translation.y() - theRobotPose.translation.y()) > p.startTranslationThreshold)) ||
     !Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), theRobotPose.translation))
    return true;
  return false;
}

bool Forward::shouldStop(const Pose2f& target) const
{
  // TODO: Remove unnecessary brackets
  //TODO: translation and rating should not be combined by conjugation but instead by disjunction
  if((rating(target.translation) - rating(theRobotPose.translation) < p.stopThreshold &&
      (std::abs(Angle::normalize(theRobotPose.rotation - (ball - theRobotPose.translation).angle())) < p.stopAngle &&
       std::abs(target.translation.x() - theRobotPose.translation.x()) < p.stopTranslationThreshold &&
       std::abs(target.translation.y() - theRobotPose.translation.y()) < p.stopTranslationThreshold)) &&
     Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), theRobotPose.translation))
    return true;
  return false;
}

void Forward::reset()
{
  changedRole = true;
}

void Forward::gradientAscent(Vector2f& pos, int numIterations) const
{
  float d = p.delta;
  for(int i = 0; i < numIterations; i++)
  {
    //if position is outside the voronoi region clip it
    Geometry::clipPointInsideConvexPolygon(region, pos);

    const float value = rating(pos);

    //sample points to get numerical gradient
    const Vector2f dx = Vector2f(d, 0);
    const Vector2f dy = Vector2f(0, d);

    const float df_dx = rating(pos + dx) - value;
    const float df_dy = rating(pos + dy) - value;
    Vector2f direction = Vector2f(df_dx, df_dy);
    direction.normalize();

    //increase searching distance if gradient is 0 to avoid getting stuck in flat areas
    if(direction == Vector2f(0, 0))
      d += p.delta;
    else
      d = p.delta;

    //draw probed points
    CROSS("behavior:Forward:ascent", pos.x() + dx.x(), pos.y(), 20, 5, Drawings::solidPen, ColorRGBA::red);
    CROSS("behavior:Forward:ascent", pos.x(), pos.y() + dy.y(), 20, 5, Drawings::solidPen, ColorRGBA::red);

    //take a step in the direction of the gradient
    const Vector2f oldP = pos;
    pos += direction * p.step;
    LINE("behavior:Forward:ascent", oldP.x(), oldP.y(), pos.x(), pos.y(), 50, Drawings::solidPen, ColorRGBA::yellow);
  }
}

float Forward::rating(const Vector2f& pos) const
{
  //outside of the voronoi region the rating is 0
  if(!Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), pos))
    return 0.f;

  //normal distribution around the base pose
  const float baseRating = std::exp(-0.5f * (pos - base).squaredNorm() / sqr(p.sigmaBase));

  const float passRating = thePassEvaluation.getRating(pos);
  const float goalRating = theExpectedGoals.getRating(pos);

  //the focus is an the multiplication term because all ratings need to be good at the same time. The additive term is for avoid getting stuck in large 0 areas where just one of the factors is 0 should be deleted
  return passRating * goalRating * baseRating * (1.f - p.addWeight) + (p.addWeight / 3) * (baseRating + goalRating + passRating);
}

void Forward::drawRating()
{
  cellColors.clear();
  gridCornerUpper = Vector2f(std::max_element(region.cbegin(), region.cend(), [](const Vector2f a, const Vector2f b) {return a.x() < b.x(); })->x(), std::max_element(region.cbegin(), region.cend(), [](const Vector2f a, const Vector2f b) {return a.y() < b.y(); })->y());
  gridCornerLower = Vector2f(std::min_element(region.cbegin(), region.cend(), [](const Vector2f a, const Vector2f b) {return a.x() < b.x(); })->x(), std::min_element(region.cbegin(), region.cend(), [](const Vector2f a, const Vector2f b) {return a.y() < b.y(); })->y());
  cellsNumber = ((gridCornerUpper - gridCornerLower).array() / Vector2f(p.cellSize, p.cellSize).array()).cast<int>();
  cellsNumber += Vector2i(1, 1);
  totalCellsNumber = cellsNumber.x() * cellsNumber.y();
  cellColors.clear();
  cellColors.reserve(totalCellsNumber);
  float nMin = 1;
  float nMax = 0;
  for(float y = gridCornerLower.y(); y <= gridCornerUpper.y(); y += p.cellSize)
  {
    for(float x = gridCornerLower.x(); x <= gridCornerUpper.x(); x += p.cellSize)
    {
      // Get combined rating at each cell in the target area on the field
      float value = rating(Vector2f(x, y));

      if(value < nMin)
        nMin = value;
      if(value > nMax)
        nMax = value;

      value = minMax.limit(value);
      value = mapToRange(value, minMax.min, minMax.max, 0.f, 1.f);

      ColorRGBA cellColor = p.worstEvaluationColor.interpolate(value, p.bestEvaluationColor);
      cellColor.a = p.heatmapAlpha;

      if(value == 0)
        cellColor.a = 0;
      cellColors.push_back(cellColor);
    }
  }
  minMax.min = nMin;
  minMax.max = nMax;
  GRID_RGBA("behavior:Forward:heatmap", (gridCornerLower.x() + gridCornerUpper.x()) / 2, (gridCornerLower.y() + gridCornerUpper.y()) / 2, p.cellSize, cellsNumber.x(), cellsNumber.y(), cellColors.data());
}

