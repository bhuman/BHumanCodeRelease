/**
 * @file FieldRatingProvider.cpp
 * @author Philip Reichenberg
 */

#include "FieldRatingProvider.h"
#include "Debugging/DebugDrawings.h"

MAKE_MODULE(FieldRatingProvider);

FieldRatingProvider::FieldRatingProvider()
{
  attractRange = std::sqrt(sqr(theFieldDimensions.xPosOpponentGoalLine * 2.f) + sqr(theFieldDimensions.yPosLeftTouchline));
  betterGoalAngleRange = theFieldDimensions.xPosOpponentGoalLine - theFieldDimensions.xPosOpponentPenaltyArea;
  drawMinX = -theFieldDimensions.xPosOpponentGoalLine;
  drawMaxX = -drawMinX;
  drawMinY = -theFieldDimensions.yPosLeftTouchline;
  drawMaxY = -drawMinY;

  rightInnerGoalPost = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightGoal + theFieldDimensions.goalPostRadius);
  leftInnerGoalPost = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius);

  rightInnerGoalPostY = rightInnerGoalPost.y();
  leftInnerGoalPostY = leftInnerGoalPost.y();
  outerGoalPostX = theFieldDimensions.xPosOpponentGoalLine - 2.f * theFieldDimensions.goalPostRadius;

  rightGoalPost.center = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightGoal);
  rightGoalPost.left = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightGoal + theFieldDimensions.goalPostRadius);
  rightGoalPost.right = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightGoal - theFieldDimensions.goalPostRadius);

  leftGoalPost.center = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftGoal);
  leftGoalPost.left = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftGoal + theFieldDimensions.goalPostRadius);
  leftGoalPost.right = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius);

  lastTeammateUpdate = 0;
  lastObstacleOnFieldUpdate = 0;

  updateParameters();
}

void FieldRatingProvider::updateParameters()
{
  fieldBorderRTV = fieldBorderValue / fieldBorderRange;
  attractRTV = attractValue / attractRange;
  teammateRTV = teammateValue / teammateAttractRange;
  passRTV = passTargetValue / passAttractRange;
  betterGoalAngleRTV = betterGoalAngleValue / betterGoalAngleRange;
  ballRTV = ballRating / ballRange;
  obstacleBackRTV = 1.f / opponentBackRangeY * opponentBackValue;
  bestBallPositionRange = Rangef(bestDistanceForBall - bestDistanceWidth, bestDistanceForBall + bestDistanceWidth);
  minXCoordinateForPass = theFieldDimensions.xPosOwnGoalLine / 2.f - interpolationZoneInOwnHalf;
}

void FieldRatingProvider::update(FieldRating& fieldRating)
{
  ASSERT(interpolationZoneInOwnHalf > 0.f);

  fieldRating.potentialFieldOnly = [this](const float x, const float y, const bool calculateFieldDirection)
  {
    PotentialValue pv;
    pv += getFieldBorderPotential(x, y, calculateFieldDirection);
    pv += getGoalPotential(x, y, calculateFieldDirection);
    pv += getGoalAnglePotential(x, y, calculateFieldDirection);
    return pv;
  };

  fieldRating.getObstaclePotential = [this](PotentialValue& pv, const float x, const float y, const bool calculateFieldDirection)
  {
    pv += getObstaclePotential(x, y, calculateFieldDirection);
  };

  fieldRating.potentialOverall = [this](PotentialValue& pv, const float x, const float y, bool& teammateArea, const bool calculateFieldDirection, const int passTarget)
  {
    lastRequestedPassTarget = passTarget;
    PotentialValue teammatePV = getTeammatesPotential(x, y, calculateFieldDirection, passTarget);
    teammateArea = teammatePV.value < 0.f;
    pv += teammatePV;
  };

  fieldRating.duelBallNearPotential = [this](PotentialValue& pv, const float x, const float y, const bool calculateFieldDirection)
  {
    pv += getBallNearPotential(x, y, calculateFieldDirection);
  };

  fieldRating.removeBallNearFromTeammatePotential = [](PotentialValue& pv, const PotentialValue& ballNear)
  {
    if(pv.value < 0.f)
      pv.value -= std::max(0.f, ballNear.value);
  };

  fieldRating.getPossiblePassTargets = [this]()
  {
    return getPossiblePassTargets();
  };

  DECLARE_DEBUG_DRAWING("module:FieldRatingProvider:potentialField", "drawingOnField");

  MODIFY("module:FieldRatingProvider:modifyDrawingRating", modifyDrawingRating);
  MODIFY("module:FieldRatingProvider:minMaxDrawing", drawMinMax);
  MODIFY("module:FieldRatingProvider:passTarget", lastRequestedPassTarget);
  MODIFY("module:FieldRatingProvider:drawing:field", fieldBorderDrawing);
  MODIFY("module:FieldRatingProvider:drawing:goal", goalDrawing);
  MODIFY("module:FieldRatingProvider:drawing:goalAngle", goalAngleDrawing);
  MODIFY("module:FieldRatingProvider:drawing:opponent", opponentDrawing);
  MODIFY("module:FieldRatingProvider:drawing:teammate", teammateDrawing);
  MODIFY("module:FieldRatingProvider:drawing:ballNear", ballNearDrawing);
  MODIFY("module:FieldRatingProvider:drawing:passTargets", passTargetDrawing);
  DEBUG_RESPONSE("module:FieldRatingProvider:drawing:activateAll")
    fieldBorderDrawing = goalDrawing = goalAngleDrawing = opponentDrawing = teammateDrawing = ballNearDrawing = passTargetDrawing = true;
  DEBUG_RESPONSE("module:FieldRatingProvider:potentialField")
    draw();
  DEBUG_RESPONSE("module:FieldRatingProvider:updateParameters")
    updateParameters();
}

float FieldRatingProvider::functionLinear(const float distance, const float radius, const float radiusTimesValue)
{
  return std::max((radius - distance) * radiusTimesValue, 0.f);
}

Vector2f FieldRatingProvider::functionLinearDer(const Vector2f& distanceVector, const float distance, const float valueSign)
{
  return valueSign * distanceVector / distance;
}

float FieldRatingProvider::functionCone(const float distance, const Angle dribbleAngle, const Angle maxAngle, const float radius, const float value)
{
  if(distance > radius || dribbleAngle > maxAngle)
    return 0.f;
  return std::max(0.5f * (1.f - dribbleAngle / maxAngle) * value + 0.5f * (1.f - distance / radius) * value, 0.f); // distance / radius must be 1 - x
}

Vector2f FieldRatingProvider::functionConeDer(const Vector2f& distanceVector, const Angle dribbleAngle, const Angle maxAngle, const float radius, const float value)
{
  const float distance = distanceVector.norm();
  if(distance > radius || dribbleAngle > maxAngle)
    return Vector2f(0.f, 0.f);
  return -value / std::abs(value) * distanceVector / distance;
}

void FieldRatingProvider::draw()
{
  const bool calculateFieldDirection = true;
  const float xShift = (2.f * theFieldDimensions.xPosOpponentGoalLine) / drawGrid.x();
  const float yShift = (2.f * theFieldDimensions.yPosLeftTouchline) / drawGrid.y();
  Rangef newMinMax(0.f, 0.f);
  const float newDrawShift = -(drawMinMax.getSize() / 2.f + drawMinMax.min) ;
  const Rangef shiftedRange(drawMinMax.min + newDrawShift, drawMinMax.max + newDrawShift);
  const Rangef targetRange(-1.f, 1.f);

  const float xShiftArrow = (2.f * theFieldDimensions.xPosOpponentGoalLine) / drawGridArrow.x();
  const float yShiftArrow = (2.f * theFieldDimensions.yPosLeftTouchline) / drawGridArrow.y();
  float counterX = 0.f;
  float counterY = 0.f;
  std::vector<Vector4f> list;
  for(float y = drawMinY;
      y <= drawMaxY;
      y = y + yShift > drawMaxY && y < drawMaxY
          ? drawMaxY
          : y + yShift)
  {
    float oldY = counterY;
    counterY += drawGridArrow.y() / drawGrid.y();
    if(counterY >= 1.f && oldY >= 1.f)
      counterY -= oldY;
    for(float x = drawMinX;
        x <= drawMaxX;
        x = x + xShift > drawMaxX && x < drawMaxX
            ? drawMaxX
            : x + xShift)
    {
      float oldX = counterX;
      counterX += drawGridArrow.x() / drawGrid.x();
      PotentialValue pv;
      PotentialValue pvBallNear;
      if(fieldBorderDrawing)
        pv += getFieldBorderPotential(x, y, calculateFieldDirection);
      if(goalDrawing)
        pv += getGoalPotential(x, y, calculateFieldDirection);
      if(goalAngleDrawing)
        pv += getGoalAnglePotential(x, y, calculateFieldDirection);
      if(opponentDrawing)
        pv += getObstaclePotential(x, y, calculateFieldDirection);
      if(ballNearDrawing)
      {
        pvBallNear = getBallNearPotential(x, y, calculateFieldDirection);
        pv += pvBallNear;
      }
      if(teammateDrawing)
      {
        PotentialValue teammate = getTeammatesPotential(x, y, calculateFieldDirection, lastRequestedPassTarget);
        if(teammate.value < 0.f)
          teammate.value -= std::max(0.f, pvBallNear.value);
        pv += teammate;
      }

      newMinMax.min = std::min(pv.value, newMinMax.min);
      newMinMax.max = std::max(pv.value, newMinMax.max);
      pv.value = targetRange.scale(shiftedRange.limit(pv.value + newDrawShift), shiftedRange);
      ColorRGBA color(0, 0, 0, 0);
      if(pv.value >= 0.f)
        color = middleRatingColor.interpolate(pv.value, badRatingColor);
      else
        color = goodRatingColor.interpolate(pv.value + 1.f, middleRatingColor);

      FILLED_RECTANGLE("module:FieldRatingProvider:potentialField",
                       static_cast<int>(x - xShift / 2.f), static_cast<int>(y - yShift / 2.f),
                       static_cast<int>(x + xShift / 2.f), static_cast<int>(y + yShift / 2.f),
                       1, Drawings::noPen, ColorRGBA(), Drawings::solidBrush, color);
      if(pv.value != 0 && counterX >= 1.f && counterY >= 1.f)
      {
        counterX -= oldX;
        pv.direction /= pv.direction.norm();
        if(std::isnan(pv.value) || std::isnan(pv.direction.x()) || std::isnan(pv.direction.y()))
          continue;
        Vector4f ob;
        ob.x() = x - xShift / 2.f * (pv.direction.x() == 0.f ? 0.f : (pv.direction.x() / std::abs(pv.direction.x())));
        ob.y() = y - yShift / 2.f * (pv.direction.y() == 0.f ? 0.f : (pv.direction.y() / std::abs(pv.direction.y())));
        ob.z() = x + pv.direction.x() * xShiftArrow * arrowLength;
        ob.w() = y + pv.direction.y() * yShiftArrow * arrowLength;
        list.push_back(ob);
      }
      if(pv.value == 0.f)
        counterX = std::min(1.f - drawGridArrow.x() / drawGrid.x(), counterX);
    }
  }
  for(const Vector4f& ob : list)
  {
    ARROW("module:FieldRatingProvider:potentialField",
          ob.x(), ob.y(), ob.z(), ob.w(),
          arrowWidth, Drawings::arrow, ColorRGBA::black);
  }
  drawMinMax = newMinMax;
  lastRequestedPassTarget = -1;

  if(passTargetDrawing)
  {
    const std::vector<Vector2f> passTargets = getPossiblePassTargets();
    for(const Vector2f& target : passTargets)
      CROSS("module:FieldRatingProvider:potentialField", target.x(), target.y(), 20, 20, Drawings::line, ColorRGBA::black);
  }
}

PotentialValue FieldRatingProvider::getFieldBorderPotential(const float x, const float y, const bool calculateFieldDirection)
{
  PotentialValue pv;
  const float leftDistance = std::max(theFieldDimensions.yPosLeftTouchline - y, 0.f);
  const float rightDistance = std::max(y - theFieldDimensions.yPosRightTouchline, 0.f);
  const float backDistance = std::max(x - theFieldDimensions.xPosOwnGoalLine, 0.f);
  const float leftRightDistance = std::max(theFieldDimensions.xPosOpponentGoalLine - x, 0.f);

  // >> Left and right
  if(leftDistance < fieldBorderRange)
  {
    const float leftBorderRating = functionLinear(leftDistance, fieldBorderRange, fieldBorderRTV);
    pv.value += leftBorderRating;
    if(calculateFieldDirection)
    {
      const Vector2f leftBorderDirection = theFieldDimensions.yPosLeftTouchline <= y ? Vector2f(0.f, -1.f) : -functionLinearDer(Vector2f(0.f, leftDistance), leftDistance, 1.f);
      pv.direction += leftBorderDirection * std::abs(leftBorderRating);
    }
  }
  if(rightDistance < fieldBorderRange)
  {
    const float rightBorderRating = functionLinear(rightDistance, fieldBorderRange, fieldBorderRTV);
    pv.value += rightBorderRating;
    if(calculateFieldDirection)
    {
      const Vector2f rightBorderDirection = theFieldDimensions.yPosRightTouchline >= y ? Vector2f(0.f, 1.f) : functionLinearDer(Vector2f(0.f, rightDistance), rightDistance, 1.f);
      pv.direction += rightBorderDirection * std::abs(rightBorderRating);
    }
  }

  // >> Own back
  if(backDistance < fieldBorderRange)
  {
    const float backBorderRating = functionLinear(backDistance, fieldBorderRange, fieldBorderRTV);
    pv.value += backBorderRating;
    if(calculateFieldDirection)
    {
      const Vector2f backBorderDirection = x <= theFieldDimensions.xPosOwnGoalLine ? Vector2f(1.f, 0.f) : functionLinearDer(Vector2f(backDistance, 0.f), backDistance, 1.f);
      pv.direction += backBorderDirection * std::abs(backBorderRating);
    }
  }

  // >> Opponent front
  if(leftRightDistance < fieldBorderRange)
  {
    const float leftRightFrontBorderRating = y > theFieldDimensions.yPosLeftGoal - 1.5f * theFieldDimensions.goalPostRadius || y < theFieldDimensions.yPosRightGoal + 1.5f * theFieldDimensions.goalPostRadius ? functionLinear(leftRightDistance, fieldBorderRange, fieldBorderRTV) : 0.f;
    pv.value += leftRightFrontBorderRating;
    if(calculateFieldDirection)
    {
      const Vector2f leftRightFrontBorderDirection = theFieldDimensions.xPosOpponentGoalLine > x && (y > theFieldDimensions.yPosLeftGoal || y < theFieldDimensions.yPosRightGoal) ? -functionLinearDer(Vector2f(leftRightDistance, 0.f), leftRightDistance, 1.f) : Vector2f(-1.f, 0.f);
      pv.direction += leftRightFrontBorderDirection * std::abs(leftRightFrontBorderRating);
    }
  }
  if(pv.value > fieldBorderValue)
  {
    pv.value = fieldBorderValue;
    if(calculateFieldDirection)
      pv.direction.normalize(fieldBorderValue);
  }
  return pv;
}

PotentialValue FieldRatingProvider::getObstaclePotential(const float x, const float y, const bool calculateFieldDirection)
{
  float obstacleRepelRating = 0.f;
  Vector2f obstacleRepelDirection(0.f, 0.f);
  const Vector2f fieldPoint(x, y);
  if(x >= theFieldDimensions.xPosOpponentGoalLine && y < theFieldDimensions.yPosLeftGoal && y > theFieldDimensions.yPosRightGoal) // the goal is always good
    return PotentialValue();

  if(lastObstacleOnFieldUpdate != theFrameInfo.time)
  {
    lastObstacleOnFieldUpdate = theFrameInfo.time;
    obstaclesOnField.clear();
    for(const Obstacle& o : theObstacleModel.obstacles)
    {
      const Vector2f obCenter = theRobotPose * o.center;
      if(std::abs(obCenter.x()) > theFieldDimensions.xPosOpponentGoalLine)
        continue;
      obstaclesOnField.push_back({ obCenter, false });
    }
    obstaclesOnField.push_back({ leftGoalPost.center, true });
    obstaclesOnField.push_back({ rightGoalPost.center, true });
  }

  const bool updateFieldPosition = lastTeammateInFieldUpdate != theFrameInfo.time;
  if(updateFieldPosition)
    teammateInField.clear();
  lastTeammateInFieldUpdate = theFrameInfo.time;
  std::size_t index = 0;
  float squaredDistance = (theRobotPose.translation - fieldPoint).squaredNorm();
  for(const auto& teammate : theGlobalTeammatesModel.teammates)
  {
    if(updateFieldPosition)
      teammateInField.push_back(teammate.getFuturePosition(estimateTeammateIntoFuture));
    const Vector2f& teammatePosition = teammateInField[index];
    const float newSquaredDistance = (teammatePosition - fieldPoint).squaredNorm();
    if(newSquaredDistance < squaredDistance)
      squaredDistance = newSquaredDistance;
    index++;
  }

  const Vector2f fieldPointToGoal = Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f) - fieldPoint;
  const float distanceToGoalFactor = std::min(1.f, fieldPointToGoal.norm() / opponentDistanceToGoal);
  // useMaxRepelRange describes the range between the min and the max value. the max radius is determined by the distance overlap of vectorToOb and squaredDistance.
  // the radius around the obstacle, that contains the max value is determined by the distance overlap minus useMaxRepelRange
  const float useMaxRepelRange = distanceToGoalFactor * minRepelDifferenceRange + (1.f - distanceToGoalFactor) * maxRepelDifferenceRange;
  const float teammateDistance = std::sqrt(squaredDistance);
  const float radiusTimesValue = 1.f / useMaxRepelRange * repelValue;
  for(const auto& ob : obstaclesOnField)
  {
    Vector2f vectorToOb = ob.position - fieldPoint;
    if(vectorToOb == Vector2f(0.f, 0.f))
      vectorToOb.x() += 0.00001f;
    float vectorToObNorm = vectorToOb.squaredNorm();
    if(vectorToObNorm > squaredDistance)
      continue;

    vectorToObNorm = std::sqrt(vectorToObNorm);
    const float malusForOpponentTurn = std::min(1.f, std::max(0.f, std::abs(vectorToOb.angle()) - 60_deg) / 90_deg);
    const float realRadius = (vectorToObNorm + (ob.isGoalPost ? std::min(goalPostMaxRadius, teammateDistance) : teammateDistance)) * selfAndOpponentShiftFactor  / 2.f * (1.f - malusForOpponentTurn * malusForOpponentTurnFactor);
    const float relativeRadius = realRadius - useMaxRepelRange;
    const float realDistance = std::max(0.f, vectorToObNorm - relativeRadius);
    float singleObstacleRating = functionLinear(realDistance, useMaxRepelRange, radiusTimesValue);

    // Behind the obstacle shall be a weak field, to discourage end positions behind and on the other y-axis side of the obstacle
    const Vector2f obShiftFieldPoint(ob.position.x(), ob.position.y() + (ob.position.y() > theFieldInterceptBall.interceptedEndPositionOnField.y() ? opponentBackShiftY : -opponentBackShiftY));
    if((fieldPoint.y() - obShiftFieldPoint.y()) * (theFieldInterceptBall.interceptedEndPositionOnField.y() - obShiftFieldPoint.y()) < 0 && ob.position.x() < fieldPoint.x())
    {
      const float dis = std::abs(fieldPoint.y() - obShiftFieldPoint.y());
      const float maxRatingRatio = Rangef::ZeroOneRange().limit(std::abs(theFieldInterceptBall.interceptedEndPositionOnField.y() - ob.position.y()) / opponentBackShiftY);
      singleObstacleRating = std::max(singleObstacleRating, maxRatingRatio * (opponentBackValue - functionLinear(dis, opponentBackRangeY, obstacleBackRTV)) * Rangef::ZeroOneRange().limit(((obShiftFieldPoint.x() + opponentBackRangeX) - theFieldInterceptBall.interceptedEndPositionOnField.x()) / opponentBackRangeX));
    }

    obstacleRepelRating += singleObstacleRating;
    if(calculateFieldDirection)
    {
      const Vector2f singleObstacleDirection = ob.position != fieldPoint && singleObstacleRating != 0.f ? vectorToOb.normalized(-1.f) : Vector2f(0.f, 0.f);
      obstacleRepelDirection += singleObstacleDirection * std::abs(singleObstacleRating);
    }
  }

  PotentialValue pv;
  pv.value = obstacleRepelRating;
  if(calculateFieldDirection)
    pv.direction = obstacleRepelDirection;
  return pv;
}

PotentialValue FieldRatingProvider::getGoalPotential(const float x, const float y, const bool calculateFieldDirection)
{
  const bool yInGoal = y < theFieldDimensions.yPosLeftGoal && y > theFieldDimensions.yPosRightGoal;
  const bool yInGoalPost = yInGoal && (y > leftInnerGoalPostY || y < rightInnerGoalPostY) && x > outerGoalPostX;

  // If a goal shot is not allowed, the "best" position shall be in front of the goal
  float goalRating;
  if(theIndirectKick.allowDirectKick)
  {
    goalRating = yInGoal && !yInGoalPost
                 ? (x > theFieldDimensions.xPosOpponentGoalLine ? attractRange * -2.f* attractRTV : -functionLinear(theFieldDimensions.xPosOpponentGoalLine - x, attractRange, attractRTV))
                 : (!yInGoal && theFieldDimensions.xPosOpponentGoalLine - x <= fieldBorderRange ? 0.f : -functionLinear((Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius) - Vector2f(x, std::abs(y))).norm(), attractRange, attractRTV));
  }
  else
    goalRating = -functionLinear((Vector2f(theFieldDimensions.xPosOpponentGoalLine + indirectGoalOffset, 0.f) - Vector2f(x, y)).norm(), attractRange, attractRTV);

  PotentialValue pv;
  pv.value = goalRating;

  if(calculateFieldDirection)
  {
    Vector2f vectorToGoal;
    if(theIndirectKick.allowDirectKick)
    {
      vectorToGoal = !yInGoal ? (y < 0.f ? rightInnerGoalPost : leftInnerGoalPost) - Vector2f(x, y) :
                     (yInGoal && !yInGoalPost
                      ? (x >= theFieldDimensions.xPosOpponentGoalLine ? Vector2f(1.f, 0.f) : Vector2f(theFieldDimensions.xPosOpponentGoalLine - x, 0.f))
                      : (y > 0.f ? Vector2f(0.f, -1.f) : Vector2f(0.f, 1.f)));
    }
    else
      vectorToGoal = Vector2f(theFieldDimensions.xPosOpponentGoalLine + indirectGoalOffset, 0.f) - Vector2f(x, y);
    if(vectorToGoal == Vector2f(0.f, 0.f))
      vectorToGoal.x() += 0.000001f;
    const Vector2f goalDirection = functionLinearDer(vectorToGoal, vectorToGoal.norm(), 1.f);
    pv.direction = goalDirection * std::abs(goalRating);
  }
  return pv;
}

PotentialValue FieldRatingProvider::getGoalAnglePotential(const float x, const float y, const bool calculateFieldDirection)
{
  // Better goal kick angle, this moves the robot back toward the own field side, when close to the touchline beside the opponent goal
  const bool yInGoal = y < theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius && y > theFieldDimensions.yPosRightGoal + theFieldDimensions.goalPostRadius;
  const float goalAngleRating = x > theFieldDimensions.xPosOpponentPenaltyArea && !yInGoal ? functionLinear(theFieldDimensions.xPosOpponentGoalLine - x, betterGoalAngleRange, betterGoalAngleRTV) :
                                0.f;

  PotentialValue pv;
  pv.value = goalAngleRating;
  if(calculateFieldDirection)
  {
    const Vector2f goalAngleDirection = x > theFieldDimensions.xPosOpponentPenaltyArea && (y >= theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius || y <= theFieldDimensions.yPosRightGoal + theFieldDimensions.goalPostRadius)
                                        ? functionLinearDer(Vector2f(theFieldDimensions.xPosOpponentPenaltyArea - x, 0.f), x - theFieldDimensions.xPosOpponentPenaltyArea, 1.f)
                                        : Vector2f(0.f, 0.f);
    pv.direction = goalAngleDirection * std::abs(goalAngleRating);
  }
  return pv;
}

void FieldRatingProvider::updateTeammateData(const Vector2f& useBallPose)
{
  if(lastTeammateUpdate != theFrameInfo.time)
  {
    lastTeammateUpdate = theFrameInfo.time;
    teammateAngleToGoal.clear();
    teammateOffsetToGoal.clear();
    teammateRangeInterpolation.clear();
    obstacleOnField.clear();
    teammateBallAngle.clear();
    const bool updateFieldPosition = lastTeammateInFieldUpdate != theFrameInfo.time
                                     || teammateInField.size() != theGlobalTeammatesModel.teammates.size();
    if(updateFieldPosition)
      teammateInField.clear();
    lastTeammateInFieldUpdate = theFrameInfo.time;
    const Vector2f goal(theFieldDimensions.xPosOpponentGoalLine, 0.f);
    std::size_t index = 0;
    for(const auto& teammate : theGlobalTeammatesModel.teammates)
    {
      ASSERT(index < theGlobalTeammatesModel.teammates.size());
      if(updateFieldPosition)
        teammateInField.push_back(teammate.getFuturePosition(estimateTeammateIntoFuture));
      const Vector2f& playerPosition = teammateInField[index];
      teammateAngleToGoal.push_back((goal - playerPosition).angle());
      teammateOffsetToGoal.push_back(playerPosition + Vector2f::polar(bestRelativePose, (goal - playerPosition).angle()));
      teammateRangeInterpolation.push_back(std::min(1.f, std::max(0.f, (playerPosition - useBallPose).norm() - minTeammatePassDistance) / maxTeammatePassDistance));
      teammateBallAngle.push_back((teammateOffsetToGoal.back() - theFieldInterceptBall.interceptedEndPositionOnField).angle());
      index++;
    }
    for(const Obstacle& ob : theObstacleModel.obstacles)
      obstacleOnField.push_back(theRobotPose * ob.center);
  }
}

PotentialValue FieldRatingProvider::getTeammatesPotential(const float x, const float y, const bool calculateFieldDirection, const int passTarget)
{
  Vector2f clippedBallPose = theFieldInterceptBall.interceptedEndPositionOnField;
  clippedBallPose.x() = std::max(clippedBallPose.x(), minXCoordinateForPass);
  // calculate some values only once per frame
  updateTeammateData(clippedBallPose);

  PotentialValue pv;
  if(x >= theFieldDimensions.xPosOpponentGoalLine && y < theFieldDimensions.yPosLeftGoal && y > theFieldDimensions.yPosRightGoal) // the goal is always good
    return pv;
  std::size_t index = -1;
  const Vector2f fieldPoint(x, y);
  const float ownHalfInterpolation = mapToRange(x, minXCoordinateForPass, minXCoordinateForPass + 2.f * interpolationZoneInOwnHalf, 0.f, 1.f);
  for(const auto& teammate : theGlobalTeammatesModel.teammates)
  {
    index++;
    ASSERT(index < theGlobalTeammatesModel.teammates.size());
    const Vector2f& playerPosition = teammateInField[index];
    const bool isPassTarget = teammate.playerNumber == passTarget;
    ASSERT(index < teammateAngleToGoal.size());
    if(!isPassTarget && playerPosition.x() < minXCoordinateForPass)
      continue;
    const Angle anglePlayerToPoint = (fieldPoint - playerPosition).angle();
    if(!isPassTarget && (std::abs(anglePlayerToPoint - teammateAngleToGoal[index]) > 90_deg || x < clippedBallPose.x())) // to safe some computation time
      continue;

    Vector2f poseVector = teammateOffsetToGoal[index] - fieldPoint;
    if(poseVector == Vector2f(0.f, 0.f))
      poseVector.x() += 0.000001f;
    // Endpositions that are on the other side of the teammate than the ball currently is are better. Scale their range to be higher
    const float poseDistance = poseVector.norm();
    const Angle fieldPointAngle = (-poseVector).angle();

    auto scaleAttractRangeWithAngle = [this](const Angle fieldPointAngle, const std::size_t index)
    {
      const Angle diffAngle = std::abs(teammateBallAngle[index] - fieldPointAngle);
      if(diffAngle > 45_deg)
        return Rangef::ZeroOneRange().limit((diffAngle - 45_deg) / 90_deg) * 0.5f + 0.5f;
      else
        return Rangef::ZeroOneRange().limit(diffAngle / 90_deg);
    };

    const float ratioAttractRange = scaleAttractRangeWithAngle(fieldPointAngle, index);
    const float useTeammateAttractRange = teammateAttractRange * (1.f - ratioAttractRange) + teammateAttractRangeMin * ratioAttractRange;
    float rating;
    // Different ratings based on if teammate is pass target
    if(!isPassTarget)
      rating = -functionLinear(poseDistance, useTeammateAttractRange, teammateRTV);
    else
    {
      const float usePassRange = useTeammateAttractRange * ratioAttractRange + passAttractRange * (1.f - ratioAttractRange);
      rating = -functionLinear(poseDistance, usePassRange, passRTV);
    }
    rating *= ownHalfInterpolation;
    if(rating < 0.f)
    {
      // Smooth out edges
      float opponentHalfScaling = -0.5f * Rangef::ZeroOneRange().limit(-rating / 0.05f);
      if(x <= 0.f)
      {
        const float ownHalfXScaling = Rangef::ZeroOneRange().limit(x / -750.f);
        opponentHalfScaling *= Rangef::ZeroOneRange().limit((teammateInField[index].x() - clippedBallPose.x()) / 750.f) * ownHalfXScaling + (1.f - ownHalfXScaling);
      }
      rating += teammateRangeInterpolation[index] * opponentHalfScaling;
      rating *= (mapToRange(((clippedBallPose - fieldPoint).norm() - (playerPosition - fieldPoint).norm()), minTeammatePassDistance, maxTeammatePassDistance, 0.f, 1.f));
      if(!isPassTarget)
      {
        float shortestDistance = std::numeric_limits<float>().max();
        for(const Vector2f& ob : obstacleOnField)
          shortestDistance = std::min(shortestDistance, (fieldPoint - ob).squaredNorm());
        if(shortestDistance < sqr(teammateMinDistanceToObstacle))
          rating *= Rangef::ZeroOneRange().limit(std::sqrt(shortestDistance / sqr(teammateMinDistanceToObstacle)));
      }
      if(x > clippedBallPose.x() - 300.f)
        rating *= Rangef::ZeroOneRange().limit((x - clippedBallPose.x()) / 300.f);
    }
    pv.value += rating;
    if(calculateFieldDirection)
    {
      const Vector2f direction = functionLinearDer(poseVector, poseDistance, 1.f);
      pv.direction += direction * std::abs(rating);
    }
  }
  return pv;
}

PotentialValue FieldRatingProvider::getBallNearPotential(const float x, const float y, const bool calculateFieldDirection)
{
  PotentialValue pv;
  if(x >= theFieldDimensions.xPosOpponentGoalLine && y < theFieldDimensions.yPosLeftGoal && y > theFieldDimensions.yPosRightGoal) // the goal is always good
    return pv;
  const Vector2f fieldPoint(x, y);
  const Vector2f vectorToGoal = Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f) - theFieldInterceptBall.interceptedEndPositionOnField;
  const Vector2f ballVector = fieldPoint - theFieldInterceptBall.interceptedEndPositionOnField;
  const Angle ballAngle = std::abs(ballVector.angle() - vectorToGoal.angle());
  if(ballAngle < ballGoalSectorWidth)
  {
    const float ballVectorLength = ballVector.norm();
    if(bestBallPositionRange.isInside(ballVectorLength))
    {
      pv.value = -ballRating / 2.f;
      pv.direction = Vector2f(0.f, 0.f);
    }
    else
    {
      const float useBestDistanceForBall = bestBallPositionRange.min > ballVectorLength ? bestBallPositionRange.min : bestBallPositionRange.max;
      const float ballNearRating = ballRating - functionLinear(std::abs(useBestDistanceForBall - ballVectorLength), ballRange, ballRTV);
      pv.value = ballNearRating - ballRating / 2.f;
      if(calculateFieldDirection)
      {
        const Vector2f bestBallPositionRelative = theFieldInterceptBall.interceptedEndPositionOnField - fieldPoint;
        const Vector2f ballDirection = bestBallPositionRelative == Vector2f(0.f, 0.f) ? bestBallPositionRelative : bestBallPositionRelative.normalized(1.f);
        pv.direction = ballDirection;
      }
    }
    // new the max allowed dribble angle "ballGoalSectorWidth", all values shall be go down to 0
    const Angle reduceRatingAtMinAngle = ballGoalSectorWidth - ballGoalSectorBorderWidth;
    if(ballAngle > reduceRatingAtMinAngle)
    {
      const float reduceValueAtBorderFactor = Rangef::ZeroOneRange().limit((ballAngle - reduceRatingAtMinAngle) / ballGoalSectorBorderWidth);
      pv.value = ballRating / 2.f * reduceValueAtBorderFactor + pv.value * (1.f - reduceValueAtBorderFactor);
    }
    pv.direction *= pv.value;
    return pv;
  }
  pv.value = ballRating / 2.f;
  return pv;
}

std::vector<Vector2f> FieldRatingProvider::getPossiblePassTargets()
{
  Vector2f clippedBallPose = theFieldInterceptBall.interceptedEndPositionOnField;
  clippedBallPose.x() = std::max(clippedBallPose.x(), minXCoordinateForPass);
  // calculate some values only once per frame
  updateTeammateData(clippedBallPose);

  std::vector<Vector2f> passTargets;
  for(std::size_t index = 0; index < theGlobalTeammatesModel.teammates.size(); index++)
  {
    const Vector2f& playerPosition = teammateInField[index];
    if(teammateRangeInterpolation[index] == 0.f || playerPosition.x() < minXCoordinateForPass || playerPosition.x() < clippedBallPose.x() - (teammateAttractRangeMin + bestRelativePose))
      continue;

    const Vector2f passTarget(std::max(clippedBallPose.x() + 300.f, playerPosition.x() + bestRelativePose), playerPosition.y());
    if((passTarget - clippedBallPose).norm() - (passTarget - playerPosition).norm() < minTeammatePassDistance)
      continue;

    passTargets.emplace_back(passTarget);
  }

  return passTargets;
}
