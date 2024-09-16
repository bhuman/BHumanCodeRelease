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
  MODIFY("behavior:RatingRole", p);
  MODIFY("behavior:RatingRole:drawHeatmap", drawHeatmap);
}

Pose2f RatingRole::position(Side, const Pose2f& basePose, const std::vector<Vector2f>& baseArea, const Agent& self, const Agents&)
{
  //initialize parameters needs to be done before every thing else!
  if(changedRole)
    pos = theRobotPose.translation;
  changedRole = false;
  base = basePose.translation;
  region = baseArea;
  ball = theFieldBall.recentBallPositionOnField();
  agent = self;

  if(drawHeatmap)
    drawRating();

  Vector2f randomPos = Vector2f(Random::normal(base.x(), p.baseDeviation), Random::normal(base.y(), p.baseDeviation));
  Geometry::clipPointInsideConvexPolygon(region, randomPos);

  // Get pos with best rating
  const float oldPosRating = rating(pos) * p.lastPosBonus;
  const float robotPoseRating = rating(theRobotPose.translation);
  const float randomPoseRating = rating(randomPos);
  pos = oldPosRating > robotPoseRating && oldPosRating > randomPoseRating ?
        pos :
        (robotPoseRating > randomPoseRating ? theRobotPose.translation : randomPos);

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
  float startThreshold = p.startThreshold;
  if(!theIndirectKick.allowDirectKick)
  {
    startThreshold = 0.f;
  }
  const float targetRating = rating(target.translation);

  //avoid division by zero
  if(targetRating == 0)
    return false;

  // rating && (position diff)
  if(((targetRating - rating(theRobotPose.translation)) / targetRating > startThreshold &&
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
  const float targetRating = rating(target.translation);

  //avoid division by zero
  if(targetRating == 0)
    return false;

  // rating || (position diff)
  if(((targetRating - rating(theRobotPose.translation)) / targetRating < p.stopThreshold ||
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
  const Vector2f originalPos = pos;
  float ratio = 1.f;
  float d = p.delta;
  const float ratioReduction = 1.f / numIterations;
  CROSS("behavior:RatingRole:ascent", pos.x(), pos.y(), 20, 5, Drawings::solidPen, ColorRGBA::gray);
  for(int i = 0; i < numIterations; i++)
  {
    const float drawRatio = 1.f - (i + 1) / static_cast<float>(numIterations) * 0.5f;
    //if position is outside the Voronoi region clip it
    Geometry::clipPointInsideConvexPolygon(region, pos);

    const float value = rating(pos);

    //sample points in the direction of the base pose to make sure they are inside the Voronoi region
    const bool sampleUp = pos.x() < base.x();
    const bool sampleLeft = pos.y() < base.y();

    //sample points to get numerical gradient
    const Vector2f dx = Vector2f(sampleUp ? d : -d, 0);
    const Vector2f dy = Vector2f(0, sampleLeft ? d : -d);

    const float df_dx = rating(pos + dx) - value;
    const float df_dy = rating(pos + dy) - value;
    Vector2f direction = Vector2f(sampleUp ? df_dx : -df_dx, sampleLeft ? df_dy : -df_dy);
    direction.normalize();

    //increase searching distance if gradient is 0 to avoid getting stuck in flat areas
    if(direction == Vector2f(0, 0))
      d += p.delta;
    else
      d = p.delta;

    //draw probed points
    CROSS("behavior:RatingRole:ascent", pos.x() + dx.x(), pos.y(), 20, 5, Drawings::solidPen, ColorRGBA(static_cast<unsigned char>(drawRatio * 255), static_cast<unsigned char>(1.f - drawRatio * 255), 0));
    CROSS("behavior:RatingRole:ascent", pos.x(), pos.y() + dy.y(), 20, 5, Drawings::solidPen, ColorRGBA(0, static_cast<unsigned char>(1.f - drawRatio * 255), static_cast<unsigned char>(drawRatio * 255)));

    //take a step in the direction of the gradient
    const Vector2f oldP = pos;
    pos += direction * p.step * ratio;
    LINE("behavior:RatingRole:ascent", oldP.x(), oldP.y(), pos.x(), pos.y(), 5, Drawings::solidPen, ColorRGBA(255, static_cast<unsigned char>(drawRatio * 255), 0));
    ratio -= ratioReduction;
    d = p.delta * ratio;
  }

  //outside the Voronoi region, return original pos
  if(!Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), pos))
    pos = originalPos;
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
