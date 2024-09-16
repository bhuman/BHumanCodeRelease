/**
 * @file PassEvaluationProvider.cpp
 *
 * This file implements a module that evaluates a target position by estimating how likely it is that a pass of the ball would be successful.
 *
 * @author Jo Lienhoop
 */

#include "PassEvaluationProvider.h"

MAKE_MODULE(PassEvaluationProvider);

PassEvaluationProvider::PassEvaluationProvider()
{
  // Create a grid on the field with a certain cell size for drawing the heatmap
  cellsNumber = ((opponentFieldCorner - ownFieldCorner).array() / Vector2f(cellSize, cellSize).array()).cast<int>();
  cellsNumber += Vector2i(1, 1);
  cellColors.reserve(cellsNumber.x() * cellsNumber.y());
}

void PassEvaluationProvider::update(PassEvaluation& thePassEvaluation)
{
  thePassEvaluation.getRating = [this](const Vector2f& baseOnField, const Vector2f& targetOnField, const bool isPositioning) -> float
  {
    return getRating(baseOnField, targetOnField, isPositioning);
  };

  DECLARE_DEBUG_DRAWING("module:PassEvaluationProvider:heatmap", "drawingOnField");
  MODIFY_ONCE("module:PassEvaluationProvider:calcPassTargetFree", calcPassTargetFree);
  MODIFY_ONCE("module:PassEvaluationProvider:calcPassLineFree", calcPassLineFree);
  MODIFY_ONCE("module:PassEvaluationProvider:calcPassTargetInField", calcPassTargetInField);
  MODIFY_ONCE("module:PassEvaluationProvider:calcShotLineFree", calcShotLineFree);
  MODIFY_ONCE("module:PassEvaluationProvider:calcShotDistance", calcShotDistance);
  MODIFY_ONCE("module:PassEvaluationProvider:drawHeatmap", drawHeatmap);
  MODIFY_ONCE("module:PassEvaluationProvider:drawCombinedHeatmap", drawCombinedHeatmap);
  MODIFY_ONCE("module:PassEvaluationProvider:isPositioningDrawing", isPositioningDrawing);
  if(drawHeatmap || drawCombinedHeatmap)
    draw();
}

float PassEvaluationProvider::getRating(const Vector2f& baseOnField, const Vector2f& targetOnField, const bool isPositioning)
{
  if(!theFieldDimensions.isInsideField(targetOnField))
    return 0.f;

  if(lastUpdateParameters != theFrameInfo.time)
  {
    lastUpdateParameters = theFrameInfo.time;
    updateParameters();
  }
  // Calculate the minimum distance from the opponents and teammates (separately) to the target itself as well as the line from the base to the target
  float minOpponentDistToTarget = std::numeric_limits<float>::max();
  float minOpponentDistToLine = std::numeric_limits<float>::max();
  for(const Vector2f& opponentOnField : opponentsOnField)
    updateMinDistances(baseOnField, targetOnField, opponentOnField, minOpponentDistToTarget, minOpponentDistToLine);

  // Estimated probability that no opponent would be at the pass target
  const float passTargetFree = calcPassTargetFree ? mapToRange(minOpponentDistToTarget, 0.f, opponentDistToTargetThreshold, 0.f, 1.f) : 1.f;

  // Estimated probability that no opponent would intercept the pass
  const float passLineFree = !isPositioning && calcPassLineFree ? mapToRange(minOpponentDistToLine, obstacleBlockingRadius, opponentDistToLineThreshold, 0.f, 1.f) : 1.f;

  // Estimated probability that the pass would be within the field and not go out of its boundary
  const float passTargetInField = calcPassTargetInField ? mapToRange(getDistanceToFieldBorder(targetOnField), 0.f, distToBoundaryThreshold, 0.f, 1.f) : 1.f;

  // Estimated probability that no teammate's direct shot towards the opponent's goal would be blocked
  const float shotLineFree = calcShotLineFree ? 1.f - ((1.f - isShotLineFree(baseOnField, targetOnField)) * theExpectedGoals.xG(baseOnField)) : 1.f;

  // Estimated probability that the target is within the kick range
  const float targetInRange = calcShotDistance ? isTargetInRange(baseOnField, targetOnField) : 1.f;

  // Estimated probability that the pass would be successful based on the above combined conditions
  const float combinedValue = passTargetFree * passLineFree * passTargetInField * shotLineFree * targetInRange;
  return std::max(minValue, combinedValue);
}

void PassEvaluationProvider::updateParameters()
{
  opponentsOnField.clear();
  for(const auto& obstacle : theGlobalOpponentsModel.opponents)
  {
    Vector2f obstacleOnField = obstacle.position;
    // Assume the opponent is oriented towards the ball and shift its position in an attempt to rate pass positions "in front of them" worse than behind them. The minimum ensures that the obstacle is not shifted onto the other side of the ball when the initial distance is smaller than opponentShiftToBall.
    const Vector2f obstacleToBall = theFieldBall.recentBallPositionOnField() - obstacleOnField;
    obstacleOnField += obstacleToBall.normalized(std::min(opponentShiftToBall, obstacleToBall.norm()));
    opponentsOnField.emplace_back(obstacleOnField);
  }
}

void PassEvaluationProvider::updateMinDistances(const Vector2f& baseOnField, const Vector2f& targetOnField, const Vector2f& obstacleOnField, float& minDistToTarget, float& minDistToLine) const
{
  const float distToTarget = (targetOnField - obstacleOnField).norm();
  if(distToTarget < minDistToTarget)
    minDistToTarget = distToTarget;
  const float distToLine = Geometry::getDistanceToEdge(Geometry::Line(baseOnField, (targetOnField - baseOnField)), obstacleOnField);
  if(distToLine < minDistToLine)
    minDistToLine = distToLine;
}

float PassEvaluationProvider::isShotLineFree(const Vector2f& baseOnField, const Vector2f& targetOnField) const
{
  if(baseOnField.x() > theFieldDimensions.xPosOpponentGoalLine)
    return 1.f;
  const Angle baseToPoint = (targetOnField - baseOnField).angle();
  const Angle baseToLeftGoalPost = (leftGoalPost - baseOnField).angle();
  const Angle baseToRightGoalPost = (rightGoalPost - baseOnField).angle();
  const Angle baseToLeftGoalArea = (leftGoalPost + goalPostShift - baseOnField).angle();
  const Angle baseToRightGoalArea = (rightGoalPost - goalPostShift - baseOnField).angle();
  // Interpolate the base to point angle between each goal post angle and its offset to create a transition
  const float leftSideTransition = mapToRange(baseToPoint, baseToLeftGoalPost, baseToLeftGoalArea, 0_deg, Angle(1.f));
  const float rightSideTransition = mapToRange(baseToPoint, baseToRightGoalArea, baseToRightGoalPost, Angle(1.f), 0_deg);
  // Both values equal 1.f when the point is inside the triangle formed by the base and both goal posts, 0.f when outside
  if(leftSideTransition == rightSideTransition)
    return leftSideTransition;
  // Otherwise the point is inside the transition area on the edges of the base to goal opening angle
  return leftSideTransition + rightSideTransition;
}

float PassEvaluationProvider::isTargetInRange(const Vector2f& baseOnField, const Vector2f& targetOnField) const
{
  const float maxDist = theGameState.isFreeKick() && theGameState.isForOwnTeam() ? distancePenaltyThresholdFreeKick : distancePenaltyThresholdPlaying;
  const float baseDist = (targetOnField - baseOnField).norm();
  return baseDist < maxDist ? 1.f : std::exp(-0.5f * sqr(baseDist - maxDist) / sqr(distancePenaltyDeviation)); // TODO: Change 1.f to param maxValue (almost) everywhere?
}

float PassEvaluationProvider::getDistanceToFieldBorder(const Vector2f& pointOnField) const
{
  return std::min(std::max(0.f, theFieldDimensions.xPosOpponentGoalLine - std::abs(pointOnField.x())),
                  std::max(0.f, theFieldDimensions.yPosLeftTouchline - std::abs(pointOnField.y())));
}

void PassEvaluationProvider::draw()
{
  cellColors.clear();
  for(float y = ownFieldCorner.y(); y <= opponentFieldCorner.y(); y += cellSize)
  {
    for(float x = ownFieldCorner.x(); x <= opponentFieldCorner.x(); x += cellSize)
    {
      const Vector2f position(x, y);
      float rating = getRating(theFieldBall.recentBallPositionOnField(), position, isPositioningDrawing); // * theExpectedGoals.getOpponentRating(position);
      if(drawCombinedHeatmap)
        rating *= theExpectedGoals.getRating(position, isPositioningDrawing);
      // Linear interpolation of rating in [0, 1] between the two colors for the heatmap
      ColorRGBA cellColor = worstRatingColor.interpolate(rating, bestRatingColor);
      cellColor.a = heatmapAlpha;
      cellColors.push_back(cellColor);
    }
  }
  GRID_RGBA("module:PassEvaluationProvider:heatmap", 0, 0, cellSize, cellsNumber.x(), cellsNumber.y(), cellColors.data());
}
