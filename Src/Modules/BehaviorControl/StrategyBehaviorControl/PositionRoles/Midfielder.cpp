/**
 * @file Midfielder.cpp
 *
 * This file implements the Midfielder role.
 * Tries to cover as much space as possible by staying far away from the teammates and the field border while not deviate to much from the base pose.
 * It tries to maximize a rating function that is based on the distance to the closest teammate.
 * For that it uses gradient ascent.
 *
 * @author Yannik Meinken
 */

#include "Midfielder.h"
#include "Math/Geometry.h"

void Midfielder::preProcess()
{
  DECLARE_DEBUG_DRAWING("behavior:Midfielder:position", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:Midfielder:ascent", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:Midfielder:heatmap", "drawingOnField");
  MODIFY("parameters:behavior:Midfielder", p);
  MODIFY_ONCE("behavior:Midfielder:drawHeatmap", drawHeatmap);
}

Pose2f Midfielder::position(Side, const Pose2f& basePose, const std::vector<Vector2f>& baseArea, const Agents&)
{
  //initialize parameters needs to be done before every thing else!
  if(changedRole)
    pos = theRobotPose.translation;
  changedRole = false;
  region = baseArea;
  ball = theFieldBall.recentBallPositionOnField();

  base = basePose.translation;

  // if the ball is in the opponent half shift position towards ball to quickly respond to a counter attack
  if(ball.x() > theFieldDimensions.xPosHalfWayLine && !(theGameState.isFreeKick() && theGameState.isForOwnTeam()))
  {
    base.x() = base.x() + (ball.x() - base.x()) * p.baseShiftX;
    //shift the y coordinate depending on how far the ball is in the opponent half to avoid a jump at the border that would need hysteresis
    const float yFactor = std::min(p.maxBaseShilftY, ball.x() / p.smoothBaseShiftYDistence);
    base.y() = base.y() + (ball.y() - base.y()) * yFactor;

    //if position is outside the voronoi region clip it
    Geometry::clipPointInsideConvexPolygon(region, pos);
    CROSS("behavior:Midfielder:position", base.x(), base.y(), 100, 20, Drawings::solidPen, ColorRGBA::violet);
  }

  if(drawHeatmap)
    drawRating();

  pos = rating(pos) > rating(theRobotPose.translation) ? pos : theRobotPose.translation;
  gradientAscent(pos, p.numIterations);

  CROSS("behavior:Midfielder:position", pos.x(), pos.y(), 100, 20, Drawings::solidPen, ColorRGBA::blue);

  return Pose2f((ball - pos).angle(), pos);
}

bool Midfielder::shouldStart(const Pose2f& target) const
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

bool Midfielder::shouldStop(const Pose2f& target) const
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

void Midfielder::reset()
{
  changedRole = true;
}

void Midfielder::gradientAscent(Vector2f& pos, int numIterations) const
{
  float d = p.delta;
  for(int i = 0; i < numIterations; i++)
  {
    //if position is outside the voronoi region clip it
    Geometry::clipPointInsideConvexPolygon(region, pos);

    const float value = rating(pos);

    //probe points near by to get the numerical gradient
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
    CROSS("behavior:Midfielder:ascent", pos.x() + dx.x(), pos.y(), 20, 5, Drawings::solidPen, ColorRGBA::red);
    CROSS("behavior:Midfielder:ascent", pos.x(), pos.y() + dy.y(), 20, 5, Drawings::solidPen, ColorRGBA::red);

    //take a step in the direction of the gradient
    const Vector2f oldP = pos;
    pos += direction * p.step;
    LINE("behavior:Midfielder:ascent", oldP.x(), oldP.y(), pos.x(), pos.y(), 50, Drawings::solidPen, ColorRGBA::yellow);
  }
}

float Midfielder::rating(const Vector2f pos) const
{
  //outside of the voronoi region the rating is 0
  if(!Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), pos))
    return 0.f;

  //normal distribution around the base pose
  const float baseRating = std::exp(-0.5f * (pos - base).squaredNorm() / sqr(p.sigmaBase));

  //min value with a compressed normal distribution on top
  const float ballRating = std::exp(-0.5f *  sqr((pos - ball).norm() - p.ballDistance) / sqr(p.sigmaBall)) * (1.f - p.minBallRating) + p.minBallRating;

  //get distance to next field border
  const float borderDistance = std::min(std::max(0.f, theFieldDimensions.xPosOpponentGroundLine - std::abs(pos.x())),
                                        std::max(0.f, theFieldDimensions.yPosLeftSideline - std::abs(pos.y())));

  //better rating far away from the field border
  const float borderRating = 1.f - std::exp(-0.5f * sqr(borderDistance) / sqr(p.sigmaBorder));

  //get the rating based on the nearest teammate
  float rating = 1;
  if(!theGlobalTeammatesModel.teammates.empty())
  {
    float minTeammateDistanceSquared = (pos - theGlobalTeammatesModel.teammates.cbegin()->pose.translation).squaredNorm();
    for(auto t = ++theGlobalTeammatesModel.teammates.cbegin(); t != theGlobalTeammatesModel.teammates.cend(); t++)
    {
      const float d = (pos - t->pose.translation).squaredNorm();
      if(d < minTeammateDistanceSquared)
        minTeammateDistanceSquared = d;
    }

    //better rating far away from teammates
    rating = 1.f - std::exp(-0.5f * minTeammateDistanceSquared / sqr(p.sigmaTeam));
  }

  //in case of a free kick we know that we are the attacking team so we can play offensively
  //search for free spaces to receive a pass
  if(theGameState.isFreeKick() && theGameState.isForOwnTeam())
    return baseRating * thePassEvaluation.getRating(pos) * (p.minGoalRating + (1.f - p.minGoalRating) * theExpectedGoals.getRating(pos));

  //combine the ratings
  return ballRating * rating * baseRating * borderRating;
}

void Midfielder::drawRating()
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
  GRID_RGBA("behavior:Midfielder:heatmap", (gridCornerLower.x() + gridCornerUpper.x()) / 2, (gridCornerLower.y() + gridCornerUpper.y()) / 2, p.cellSize, cellsNumber.x(), cellsNumber.y(), cellColors.data());
}

