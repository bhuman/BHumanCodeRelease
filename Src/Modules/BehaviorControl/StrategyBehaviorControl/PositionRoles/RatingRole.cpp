/**
 * @file RatingRole.cpp
 *
 * This file implements the RatingRole.
 * It tries to maximize a rating function.
 * For that it uses gradient ascent.
 *
 * @author Yannik Meinken
 */

#include "RatingRole.h"
#include "Math/Geometry.h"
#include "Math/Random.h"

void RatingRole::preProcess()
{
  DECLARE_DEBUG_DRAWING("behavior:RatingRole:position", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:RatingRole:ascent", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:RatingRole:heatmap", "drawingOnField");
  MODIFY("parameters:behavior:RatingRole", p);
  MODIFY("behavior:RatingRole:drawHeatmap", drawHeatmap);
}

Pose2f RatingRole::position(Side, const Pose2f& basePose, const std::vector<Vector2f>& baseArea, const Agents&)
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

  Vector2f randomPos = Vector2f(Random::normal(base.x(), p.baseDeviation), Random::normal(base.y(), p.baseDeviation));
  Geometry::clipPointInsideConvexPolygon(region, randomPos);

  pos = rating(pos) > rating(theRobotPose.translation) ? pos : theRobotPose.translation;
  pos = rating(pos) > rating(randomPos) ? pos : randomPos;
  gradientAscent(pos, p.numIterations);

  const Angle rotation = KickSelection::calculateTargetRotation(ball, pos, opponentGoal);
  positionBuffer.push_front(pos);
  Vector2f usePos = pos;
  if(positionBuffer.full())
    usePos = positionBuffer.average();
  CROSS("behavior:RatingRole:position", usePos.x(), usePos.y(), 100, 20, Drawings::solidPen, ColorRGBA::blue);
  ARROW("behavior:RatingRole:position", usePos.x(), usePos.y(), usePos.x() + 500 * cos(rotation), usePos.y() + 500 * sin(rotation), 20, Drawings::solidPen,
        ColorRGBA::blue);
  return Pose2f(rotation, usePos);
}

bool RatingRole::shouldStart(const Pose2f& target) const
{
  //avoid division by zero
  if(rating(target.translation) == 0)
    return false;

  // rating && (position diff)
  if(((rating(target.translation) - rating(theRobotPose.translation)) / rating(target.translation) > p.startThreshold &&
      (std::abs(target.translation.x() - theRobotPose.translation.x()) > p.startTranslationThreshold ||
       std::abs(target.translation.y() - theRobotPose.translation.y()) > p.startTranslationThreshold)) ||
     // rotation diff
     std::abs(Angle::normalize(theRobotPose.rotation - KickSelection::calculateTargetRotation(ball, theRobotPose.translation, opponentGoal))) > p.startAngle ||
     std::abs(Angle::normalize(theRobotPose.rotation - (ball - theRobotPose.translation).angle())) > p.maxAngleToBall ||
     // outside cell
     !Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), theRobotPose.translation))
    return true;
  return false;
}

bool RatingRole::shouldStop(const Pose2f& target) const
{
  //avoid division by zero
  if(rating(target.translation) == 0)
    return false;

  // rating || (position diff)
  if(((rating(target.translation) - rating(theRobotPose.translation)) / rating(target.translation) < p.stopThreshold ||
      (std::abs(target.translation.x() - theRobotPose.translation.x()) < p.stopTranslationThreshold &&
       std::abs(target.translation.y() - theRobotPose.translation.y()) < p.stopTranslationThreshold)) &&
     // rotation diff
     std::abs(Angle::normalize(theRobotPose.rotation - KickSelection::calculateTargetRotation(ball, theRobotPose.translation, opponentGoal))) < p.stopAngle &&
     std::abs(Angle::normalize(theRobotPose.rotation - (ball - theRobotPose.translation).angle())) < p.maxAngleToBall &&
     // inside cell
     Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), theRobotPose.translation))
    return true;
  return false;
}

void RatingRole::reset()
{
  changedRole = true;
  positionBuffer.reserve(p.positionBufferSize);
  positionBuffer.clear();
}

void RatingRole::gradientAscent(Vector2f& pos, int numIterations) const
{
  float d = p.delta;
  float ratio = 1.f;
  const float ratioReduction = 1.f / numIterations;
  for(int i = 0; i < numIterations; i++)
  {
    //if position is outside the voronoi region clip it
    Geometry::clipPointInsideConvexPolygon(region, pos);

    const float value = rating(pos);

    //sample points in the direction of the base pose to make sure there are inside the voronoi region
    const bool sampleUp = pos.x() < base.x();
    const bool sampleLeft = pos.y() < base.y();

    //sample points to get numerical gradient
    const Vector2f dx = Vector2f(sampleUp ? d : -d, 0);
    const Vector2f dy = Vector2f(0, sampleLeft ? d : -d);

    const float df_dx = rating(pos + dx) - value;
    const float df_dy = rating(pos + dy) - value;
    Vector2f direction = Vector2f(sampleUp ? df_dx : -df_dx, sampleLeft ? df_dy : -df_dx);
    direction.normalize();

    //increase searching distance if gradient is 0 to avoid getting stuck in flat areas
    if(direction == Vector2f(0, 0))
      d += p.delta;
    else
      d = p.delta;

    //draw probed points
    CROSS("behavior:RatingRole:ascent", pos.x() + dx.x(), pos.y(), 20, 5, Drawings::solidPen, ColorRGBA::red);
    CROSS("behavior:RatingRole:ascent", pos.x(), pos.y() + dy.y(), 20, 5, Drawings::solidPen, ColorRGBA::red);

    //take a step in the direction of the gradient
    const Vector2f oldP = pos;
    pos += direction * p.step * ratio;
    LINE("behavior:RatingRole:ascent", oldP.x(), oldP.y(), pos.x(), pos.y(), 50, Drawings::solidPen, ColorRGBA::yellow);
    ratio -= ratioReduction;
  }
}

void RatingRole::drawRating()
{
  cellColors.clear();
  gridCornerUpper = Vector2f(std::max_element(region.cbegin(), region.cend(), [](const Vector2f a, const Vector2f b)
  { return a.x() < b.x(); })->x(), std::max_element(region.cbegin(), region.cend(), [](const Vector2f a, const Vector2f b)
  { return a.y() < b.y(); })->y());
  gridCornerLower = Vector2f(std::min_element(region.cbegin(), region.cend(), [](const Vector2f a, const Vector2f b)
  { return a.x() < b.x(); })->x(), std::min_element(region.cbegin(), region.cend(), [](const Vector2f a, const Vector2f b)
  { return a.y() < b.y(); })->y());
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
  GRID_RGBA("behavior:RatingRole:heatmap", (gridCornerLower.x() + gridCornerUpper.x()) / 2, (gridCornerLower.y() + gridCornerUpper.y()) / 2, p.cellSize,
            cellsNumber.x(), cellsNumber.y(), cellColors.data());
}
