/**
 * @file FieldRatingProvider.cpp
 * @author Philip Reichenberg
 */

#include "FieldRatingProvider.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(FieldRatingProvider, behaviorControl);

FieldRatingProvider::FieldRatingProvider()
{
  attractRange = std::sqrt(sqr(theFieldDimensions.xPosOpponentGroundLine * 2.f) + sqr(theFieldDimensions.yPosLeftSideline));
  betterGoalAngleRange = theFieldDimensions.xPosOpponentGroundLine - theFieldDimensions.xPosOpponentPenaltyArea;
  drawMinX = -theFieldDimensions.xPosOpponentGroundLine;
  drawMaxX = -drawMinX;
  drawMinY = -theFieldDimensions.yPosLeftSideline;
  drawMaxY = -drawMinY;
  otherSideYCoordinate = percentOnSide * theFieldDimensions.yPosLeftSideline;

  rightInnerGoalPost = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoal + theFieldDimensions.goalPostRadius);
  leftInnerGoalPost = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius);

  rightInnerGoalPostY = rightInnerGoalPost.y();
  leftInnerGoalPostY = leftInnerGoalPost.y();
  outerGoalPostX = theFieldDimensions.xPosOpponentGroundLine - 2.f * theFieldDimensions.goalPostRadius;
  robotRotation = Vector2f::polar(1.f, theRobotPose.rotation);

  fieldBorderDrawing = false;
  goalDrawing = false;
  goalAngleDrawing = false;
  opponentDrawing = false;
  teammateDrawing = false;
  otherSideDrawing = false;
  facingDrawing = false;
  ballNearDrawing = false;
  lastTeammateUpdate = 0;
  lastObstacleOnFieldUpdate = 0;

  updateParameters();
}

void FieldRatingProvider::updateParameters()
{
  fieldBorderRTV = fieldBorderValue / fieldBorderRange;
  attractRTV = attractValue / attractRange;
  teammateRTV = teammateValue / teammateAttractRange;
  betterGoalAngleRTV = betterGoalAngleValue / betterGoalAngleRange;
  facingRTV = facingValue / facingRange;
  ballRTV = ballRating / ballRange;
  bestBallPositionRange = Rangef(bestDistanceForBall - bestDistanceWidth, bestDistanceForBall + bestDistanceWidth);
  lowPassFilterFactor = lowPassFilterFactorPerSecond * Constants::motionCycleTime;
}

void FieldRatingProvider::update(FieldRating& fieldRating)
{
  // low pass filter hack. Otherwise dribble angle converges to 0 when walking around the ball.
  // with the hack the smalles dribble angle (when no obstacles are near), going outwards to the field sides, is about 20_deg (previous about 40_deg?)
  robotRotation = robotRotation.normalize(1.f - lowPassFilterFactor) + Vector2f::polar(lowPassFilterFactor, theRobotPose.rotation);

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

  fieldRating.potentialOverall = [this](PotentialValue& pv, const float x, const float y, bool& teammateArea, const bool calculateFieldDirection)
  {
    PotentialValue teammatePV = getTeammatesPotential(x, y, calculateFieldDirection);
    teammateArea = teammatePV.value < 0.f;
    pv += teammatePV;
  };

  fieldRating.potentialWithRobotFacingDirection = [this](PotentialValue& pv, const float x, const float y, const bool calculateFieldDirection)
  {
    pv += getRobotFacingPotential(x, y, calculateFieldDirection);
  };

  fieldRating.duelBallNearPotential = [this](PotentialValue& pv, const float x, const float y, const bool calculateFieldDirection)
  {
    pv += getBallNearPotential(x, y, calculateFieldDirection);
  };

  fieldRating.getPotentialOtherSide = [this](PotentialValue& pv, const float x, const float y, const bool calculateFieldDirection)
  {
    pv += getPotentialOtherSide(x, y, calculateFieldDirection);
  };

  fieldRating.removeBallNearFromTeammatePotential = [](PotentialValue& pv, const PotentialValue& ballNear)
  {
    if(pv.value < 0.f)
      pv.value -= std::max(0.f, ballNear.value);
  };

  DECLARE_DEBUG_DRAWING("module:FieldRatingProvider:potentialField", "drawingOnField");

  MODIFY("module:FieldRatingProvider:drawing:field", fieldBorderDrawing);
  MODIFY("module:FieldRatingProvider:drawing:goal", goalDrawing);
  MODIFY("module:FieldRatingProvider:drawing:goalAngle", goalAngleDrawing);
  MODIFY("module:FieldRatingProvider:drawing:opponent", opponentDrawing);
  MODIFY("module:FieldRatingProvider:drawing:teammate", teammateDrawing);
  MODIFY("module:FieldRatingProvider:drawing:otherSide", otherSideDrawing);
  MODIFY("module:FieldRatingProvider:drawing:facing", facingDrawing);
  MODIFY("module:FieldRatingProvider:drawing:ballNear", ballNearDrawing);
  DEBUG_RESPONSE("module:FieldRatingProvider:drawing:activateAll")
    fieldBorderDrawing = goalDrawing = goalAngleDrawing = opponentDrawing = teammateDrawing = otherSideDrawing = facingDrawing = ballNearDrawing = true;
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
  return std::max(0.5f * (1.f - dribbleAngle / maxAngle) * value + 0.5f * (1.f - distance / radius) * value, 0.f); // distance / radius muss 1- x sein
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
  const float xShift = (2.f * theFieldDimensions.xPosOpponentGroundLine) / drawGrid.x();
  const float yShift = (2.f * theFieldDimensions.yPosLeftSideline) / drawGrid.y();

  for(float y = drawMinY;
      y <= drawMaxY;
      y = y + yShift > drawMaxY && y < drawMaxY
          ? drawMaxY
          : y + yShift)
  {
    for(float x = drawMinX;
        x <= drawMaxX;
        x = x + xShift > drawMaxX && x < drawMaxX
            ? drawMaxX
            : x + xShift)
    {
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
      if(facingDrawing)
        pv += getRobotFacingPotential(x, y, calculateFieldDirection);
      if(ballNearDrawing)
      {
        pvBallNear = getBallNearPotential(x, y, calculateFieldDirection);
        pv += pvBallNear;
      }
      if(teammateDrawing)
      {
        PotentialValue teammate = getTeammatesPotential(x, y, calculateFieldDirection);
        if(teammate.value < 0.f)
          teammate.value -= std::max(0.f, pvBallNear.value);
        pv += teammate;
      }
      if(otherSideDrawing)
        pv += getPotentialOtherSide(x, y, calculateFieldDirection);
      pv.value = Rangef(-1.5f, 1.f).limit(pv.value);
      ColorRGBA color(0, 0, 0, 0);
      if(pv.value >= 0.f)
      {
        color.r = static_cast<unsigned char>(pv.value * 255);
        color.b = 0;
        color.g = static_cast<unsigned char>((1.f - pv.value) * 255);
        color.a = 100;
      }
      else
      {
        color.r = 0;
        color.b = static_cast<unsigned char>(pv.value * -0.5f * 255);
        color.g = static_cast<unsigned char>((1.f - pv.value * -0.5f) * 255);
        color.a = 100;
      }

      FILLED_RECTANGLE("module:FieldRatingProvider:potentialField",
                       static_cast<int>(x - xShift / 2.f), static_cast<int>(y - yShift / 2.f),
                       static_cast<int>(x + xShift / 2.f), static_cast<int>(y + yShift / 2.f),
                       1, Drawings::noPen, ColorRGBA(), Drawings::solidBrush, color);
      if(pv.value != 0)
      {
        pv.direction /= pv.direction.norm();
        if(std::isnan(pv.value) || std::isnan(pv.direction.x()) || std::isnan(pv.direction.y()))
          continue;
        ARROW("module:FieldRatingProvider:potentialField", x, y, x + pv.direction.x() * 50.f, y + pv.direction.y() * 50.f, 5, Drawings::arrow, ColorRGBA::black);
      }
    }
  }
}

PotentialValue FieldRatingProvider::getFieldBorderPotential(const float x, const float y, const bool calculateFieldDirection)
{
  PotentialValue pv;
  const float leftDistance = std::max(theFieldDimensions.yPosLeftSideline - y, 0.f);
  const float rightDistance = std::max(y - theFieldDimensions.yPosRightSideline, 0.f);
  const float backDistance = std::max(x - theFieldDimensions.xPosOwnGroundLine, 0.f);
  const float leftRightDistance = std::max(theFieldDimensions.xPosOpponentGroundLine - x, 0.f);

  // >> Left and right
  if(leftDistance < fieldBorderRange)
  {
    const float leftBorderRating = functionLinear(leftDistance, fieldBorderRange, fieldBorderRTV);
    pv.value += leftBorderRating;
    if(calculateFieldDirection)
    {
      const Vector2f leftBorderDirection = theFieldDimensions.yPosLeftSideline <= y ? Vector2f(0.f, -1.f) : -functionLinearDer(Vector2f(0.f, leftDistance), leftDistance, 1.f);
      pv.direction += leftBorderDirection * std::abs(leftBorderRating);
    }
  }
  if(rightDistance < fieldBorderRange)
  {
    const float rightBorderRating = functionLinear(rightDistance, fieldBorderRange, fieldBorderRTV);
    pv.value += rightBorderRating;
    if(calculateFieldDirection)
    {
      const Vector2f rightBorderDirection = theFieldDimensions.yPosRightSideline >= y ? Vector2f(0.f, 1.f) : functionLinearDer(Vector2f(0.f, rightDistance), rightDistance, 1.f);
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
      const Vector2f backBorderDirection = x <= theFieldDimensions.xPosOwnGroundLine ? Vector2f(1.f, 0.f) : functionLinearDer(Vector2f(backDistance, 0.f), backDistance, 1.f);
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
      const Vector2f leftRightFrontBorderDirection = theFieldDimensions.xPosOpponentGroundLine > x && (y > theFieldDimensions.yPosLeftGoal || y < theFieldDimensions.yPosRightGoal) ? -functionLinearDer(Vector2f(leftRightDistance, 0.f), leftRightDistance, 1.f) : Vector2f(-1.f, 0.f);
      pv.direction += leftRightFrontBorderDirection * std::abs(leftRightFrontBorderRating);
    }
  }
  return pv;
}

PotentialValue FieldRatingProvider::getObstaclePotential(const float x, const float y, const bool calculateFieldDirection)
{
  float obstacleRepelRating = 0.f;
  Vector2f obstacleRepelDirection(0.f, 0.f);
  const Vector2f fieldPoint(x, y);
  if(x >= theFieldDimensions.xPosOpponentGroundLine && y < theFieldDimensions.yPosLeftGoal && y > theFieldDimensions.yPosRightGoal) // the goal is always good
    return PotentialValue();

  if(lastObstacleOnFieldUpdate != theFrameInfo.time)
  {
    lastObstacleOnFieldUpdate = theFrameInfo.time;
    obstaclesOnField.clear();
    for(const Obstacle& o : theObstacleModel.obstacles)
    {
      if(o.type == Obstacle::goalpost)
        continue;
      const Vector2f obCenter = theRobotPose * o.center;
      if(std::abs(obCenter.x()) > theFieldDimensions.xPosOpponentGroundLine)
        continue;
      obstaclesOnField.push_back(obCenter);
    }
  }

  float squardedDistance = (theRobotPose.translation - fieldPoint).squaredNorm();
  for(const Teammate& player : theTeamData.teammates)
  {
    if(player.status == Teammate::PENALIZED)
      continue;
    const float newSquaredDistance = (player.theRobotPose.translation - fieldPoint).squaredNorm();
    if(newSquaredDistance < squardedDistance)
      squardedDistance = newSquaredDistance;
  }

  const Vector2f fieldPointToGoal = Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f) - fieldPoint;
  const float distanceToGoalFactor = std::min(1.f, fieldPointToGoal.norm() / opponentDistanceToGoal);
  // useMaxRepelRange describes the range between the min and the max value. the max radius is determined by the distance overlap of vectorToOb and squardedDistance.
  // the radius around the obstacle, that contains the max value is determined by the distance overlap minus useMaxRepelRange
  const float useMaxRepelRange = distanceToGoalFactor * minRepelDifferenceRange + (1.f - distanceToGoalFactor) * maxRepelDifferenceRange;
  const float teammateDistance = std::sqrt(squardedDistance);
  const float radiusTimesValue = 1.f / useMaxRepelRange * repelValue;
  for(const Vector2f& obCenter : obstaclesOnField)
  {
    Vector2f vectorToOb = obCenter - fieldPoint;
    if(vectorToOb == Vector2f(0.f, 0.f))
      vectorToOb.x() += 0.00001f;
    float vectorToObNorm = vectorToOb.squaredNorm();
    if(vectorToObNorm > squardedDistance)
      continue;

    vectorToObNorm = std::sqrt(vectorToObNorm);
    const float malusForOpponentTurn = std::min(1.f, std::max(0.f, std::abs(vectorToOb.angle()) - 60_deg) / 90_deg);
    const float realRadius = (vectorToObNorm + teammateDistance) / 2.f * (1.f - malusForOpponentTurn * malusForOpponentTurnFactor);
    const float relativeRadius = realRadius - useMaxRepelRange;
    const float realDistance = std::max(0.f, vectorToObNorm - relativeRadius);
    const float singleObstacleRating = functionLinear(realDistance, useMaxRepelRange, radiusTimesValue);
    obstacleRepelRating += singleObstacleRating;

    if(calculateFieldDirection)
    {
      const Vector2f singleObstacleDirection = obCenter != fieldPoint && singleObstacleRating != 0.f ? vectorToOb.normalized(-1.f) : Vector2f(0.f, 0.f);
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

  const float goalRating = yInGoal && !yInGoalPost
                           ? (x > theFieldDimensions.xPosOpponentGroundLine ? attractRange * -2.f* attractRTV : -functionLinear(theFieldDimensions.xPosOpponentGroundLine - x, attractRange, attractRTV))
                           : (!yInGoal && theFieldDimensions.xPosOpponentGroundLine - x <= fieldBorderRange ? 0.f : -functionLinear((Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius) - Vector2f(x, std::abs(y))).norm(), attractRange, attractRTV));

  PotentialValue pv;
  pv.value = goalRating;

  if(calculateFieldDirection)
  {
    Vector2f vectorToGoal = !yInGoal ? (y < 0.f ? rightInnerGoalPost : leftInnerGoalPost) - Vector2f(x, y) :
                            (yInGoal && !yInGoalPost
                             ? (x >= theFieldDimensions.xPosOpponentGroundLine ? Vector2f(1.f, 0.f) : Vector2f(theFieldDimensions.xPosOpponentGroundLine - x, 0.f))
                             : (y > 0.f ? Vector2f(0.f, -1.f) : Vector2f(0.f, 1.f)));
    if(vectorToGoal == Vector2f(0.f, 0.f))
      vectorToGoal.x() += 0.000001f;
    const Vector2f goalDirection = functionLinearDer(vectorToGoal, vectorToGoal.norm(), 1.f);
    pv.direction = goalDirection * std::abs(goalRating);
  }
  return pv;
}

PotentialValue FieldRatingProvider::getGoalAnglePotential(const float x, const float y, const bool calculateFieldDirection)
{
  // Better goal kick angle, this moves the robot back toward the own field side, when close to the sideline beside the opponent goal
  const bool yInGoal = y < theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius && y > theFieldDimensions.yPosRightGoal + theFieldDimensions.goalPostRadius;
  const float goalAngleRating = x > theFieldDimensions.xPosOpponentPenaltyArea && !yInGoal ? functionLinear(theFieldDimensions.xPosOpponentGroundLine - x, betterGoalAngleRange, betterGoalAngleRTV) :
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

PotentialValue FieldRatingProvider::getTeammatesPotential(const float x, const float y, const bool calculateFieldDirection)
{
  // calculate some values only once per frame
  if(lastTeammateUpdate != theFrameInfo.time)
  {
    lastTeammateUpdate = theFrameInfo.time;
    teammateAngleToGoal.clear();
    teammateOffsetToGoal.clear();
    teammateRangeInterpolation.clear();
    teammateObstacleOnField.clear();
    const Vector2f goal(theFieldDimensions.xPosOpponentGroundLine, 0.f);
    for(const Teammate& player : theTeamData.teammates)
    {
      teammateAngleToGoal.push_back((goal - player.theRobotPose.translation).angle());
      teammateOffsetToGoal.push_back(player.theRobotPose.translation + Vector2f::polar(bestRelativePose, (goal - player.theRobotPose.translation).angle()));
      teammateRangeInterpolation.push_back(std::min(1.f, std::max(0.f, (player.theRobotPose.translation - theRobotPose.translation).norm() - minTeammatePassDistance) / maxTeammatePassDistance));
      std::vector<Vector2f> obstacles;
      for(const Obstacle& ob : player.theObstacleModel.obstacles)
        obstacles.push_back(player.theRobotPose * ob.center);
      teammateObstacleOnField.push_back(obstacles);
    }
  }
  PotentialValue pv;
  if(x >= theFieldDimensions.xPosOpponentGroundLine && y < theFieldDimensions.yPosLeftGoal && y > theFieldDimensions.yPosRightGoal) // the goal is always good
    return pv;
  size_t index = -1;
  const Vector2f fieldPoint(x, y);
  for(const Teammate& player : theTeamData.teammates)
  {
    index++;
    ASSERT(index < teammateAngleToGoal.size());
    if(player.status == Teammate::PENALIZED)
      continue;
    if(player.theRobotPose.translation.x() < theFieldDimensions.xPosOwnGroundLine / 2.f)
      continue;
    const Angle angleToBall = (fieldPoint - player.theRobotPose.translation).angle();
    if(std::abs(angleToBall - teammateAngleToGoal[index]) > 90_deg) // to safe some computation time
      continue;

    Vector2f poseVector = teammateOffsetToGoal[index] - fieldPoint;
    if(poseVector == Vector2f(0.f, 0.f))
      poseVector.x() += 0.000001f;
    const float poseDistance = poseVector.norm();
    float rating = -functionLinear(poseDistance, teammateAttractRange, teammateRTV);
    if(rating < 0.f)
    {
      if(x > 0.f)  // if pass robot is standing far away while in the opponent half, a pass would be really good
        rating += teammateRangeInterpolation[index] * -0.5f;
      for(const Vector2f& ob : teammateObstacleOnField[index])
      {
        const float squaredDistance = (fieldPoint - ob).squaredNorm();
        if(squaredDistance < sqr(1000.f))
        {
          rating = 0.f;
          break;
        }
      }
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

PotentialValue FieldRatingProvider::getPotentialOtherSide(const float x, const float y, const bool calculateFieldDirection)
{
  const float maxRating = otherSideMaxRating * std::abs(theFieldBall.endPositionOnField.y()) / otherSideYCoordinate; // no clip
  const Vector2f refSide(0.f, (theFieldBall.endPositionOnField.y() > 0.f ? -1.f : 1.f) * otherSideYCoordinate - y);
  const float refSideDistance = std::abs(refSide.y());
  const float rating = x > 0.f ? 0.f : -functionLinear(refSideDistance, otherSideWidth, 1.f / otherSideWidth * maxRating);

  PotentialValue pv;
  pv.value = rating;
  if(calculateFieldDirection)
  {
    const Vector2f direction = maxRating == 0 ? Vector2f(0.f, 0.f) : functionLinearDer(refSide, refSideDistance, 1.f);
    pv.direction = direction * std::abs(rating);
  }
  return pv;
}

PotentialValue FieldRatingProvider::getRobotFacingPotential(const float x, const float y, const bool calculateFieldDirection)
{
  Rangea faceDirectionRange(-45_deg, 45_deg);
  Angle useRobotRotation = faceDirectionRange.limit(robotRotation.angle());
  if(theRobotPose.translation.x() > theFieldBall.endPositionOnField.x())
    useRobotRotation = theRobotPose.translation.y() > theFieldBall.endPositionOnField.y() ? -45_deg : 45_deg;
  PotentialValue pv;
  const Vector2f vectorToBall = Vector2f(x, y) - (theFieldBall.endPositionOnField + Vector2f(-1.f, 0.f).rotate(useRobotRotation) * facingBackShift);
  const Angle angleFromBall = std::abs(vectorToBall.angle() - useRobotRotation);
  const float facingRating = -functionCone(vectorToBall.norm(), angleFromBall, maxFacingAngle, facingRange, facingValue);
  pv.value = facingRating;
  if(calculateFieldDirection)
    pv.direction = Vector2f(1.f, 0.f).rotate(useRobotRotation) * std::abs(facingRating);
  return pv;
}

PotentialValue FieldRatingProvider::getBallNearPotential(const float x, const float y, const bool calculateFieldDirection)
{
  PotentialValue pv;
  if(x >= theFieldDimensions.xPosOpponentGroundLine && y < theFieldDimensions.yPosLeftGoal && y > theFieldDimensions.yPosRightGoal) // the goal is always good
    return pv;
  const Vector2f fieldPoint(x, y);
  const Vector2f vectorToGoal = Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f) - theFieldBall.positionOnField;
  const Vector2f ballVector = fieldPoint - theFieldBall.positionOnField;
  if(std::abs(ballVector.angle() - vectorToGoal.angle()) < 110_deg)
  {
    const float ballVectorLength = ballVector.norm();
    if(bestBallPositionRange.isInside(ballVectorLength))
    {
      pv.value = -ballRating + ballRating / 2.f;
      pv.direction = Vector2f(0.f, 0.f);
      return pv;
    }

    const float useBestDistanceForBall = bestBallPositionRange.min > ballVectorLength ? bestBallPositionRange.min : bestBallPositionRange.max;
    const float ballNearRating = ballRating - functionLinear(std::abs(useBestDistanceForBall - ballVectorLength), ballRange, ballRTV);
    pv.value = ballNearRating - ballRating / 2.f;
    if(calculateFieldDirection)
    {
      const Vector2f bestBallPostionRelativ = ballVector / ballVectorLength * useBestDistanceForBall + theFieldBall.positionOnField - fieldPoint;
      const Vector2f ballDirection = bestBallPostionRelativ == Vector2f(0.f, 0.f) ? bestBallPostionRelativ : functionLinearDer(bestBallPostionRelativ, bestBallPostionRelativ.norm(), 1.f);
      pv.direction = ballDirection * std::abs(ballNearRating);
    }
    return pv;
  }
  else
    return pv;
}
