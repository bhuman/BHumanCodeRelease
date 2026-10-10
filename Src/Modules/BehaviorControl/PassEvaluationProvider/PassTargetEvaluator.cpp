/**
 * @file PassTargetEvaluator.cpp
 *
 * This file defines a representation that evaluates a target position for passes.
 *
 * @author Philip Reichenberg
 */

#include "PassTargetEvaluator.h"
#include "Debugging/DebugDrawings.h"
#include "Tools/Modeling/Obstacle.h"

MAKE_MODULE(PassTargetEvaluator);

PassTargetEvaluator::PassTargetEvaluator():
  opponentGoal(Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f))
{}

void PassTargetEvaluator::update(PassTargetEvaluation& thePassTargetEvaluation)
{
  DEBUG_DRAWING("module:PassTargetEvaluator:wheel", "drawingOnField")
  {
    if(lastTarget.isValid)
      DRAW_SECTOR_WHEEL("module:PassTargetEvaluator:wheel", calculateObstacleSectors(lastTarget.ballOnField, lastTarget.isAngleFree, lastTarget.teammate), lastTarget.ballOnField);
  }

  targetEvaluations.clear();

  if(resetLastTarget)
    lastTarget.isValid = false;
  else
    resetLastTarget = true;

  if(lastTarget.isValid && lastTarget.teammate != -1 && theSentTeamMessage.theBehaviorStatus.passTarget == lastTarget.teammate
     && theSentTeamMessage.theBehaviorStatus.shootingTo.has_value())
  {
    const Pose2f localChange = theRobotPose.inverse() * lastRobotPose;
    const Vector2f ballInField = localChange * lastTarget.ballOnField;
    const Vector2f shootingToInField = theSentTeamMessage.theRobotPose * theSentTeamMessage.theBehaviorStatus.shootingTo.value();
    const Angle oldKickAngle = (shootingToInField - ballInField).angle();
    const Angle lastKickAngle = (lastTarget.bestShiftedTargetOnField - ballInField).angle();
    passHysteresis = std::abs(Angle::normalize(oldKickAngle - lastKickAngle)) < keepOldTargetBearing &&
                     (shootingToInField - lastTarget.bestShiftedTargetOnField).squaredNorm() < sqr(keepOldTargetDistance);
  }
  else
    passHysteresis = false;

  thePassTargetEvaluation.getRating = [this](const Vector2f& ballOnField, const Vector2f& targetOnField, const int teammate) -> float
  {
    return getRating(ballOnField, targetOnField, teammate).passRating;
  };

  thePassTargetEvaluation.getBestPassData = [this](const Vector2f& ballOnField, const Vector2f& targetOnField, const int teammate) -> TargetEvaluation
  {
    return getRating(ballOnField, targetOnField, teammate);
  };

  thePassTargetEvaluation.setLastPassTarget = [this](const TargetEvaluation& target) -> void
  {
    lastTarget = target;
    resetLastTarget = false;
  };

  lastRobotPose = theRobotPose;
}
TargetEvaluation& PassTargetEvaluator::getRating(const Vector2f& ballOnField, const Vector2f& targetOnField, const int teammate)
{
  const auto it = std::find_if(targetEvaluations.begin(), targetEvaluations.end(), [&](const TargetEvaluation& entry)
  {return entry.ballOnField == ballOnField && entry.targetOnField == targetOnField && entry.teammate == teammate; });
  if(it != targetEvaluations.end())
    return *it;

  Vector2f teammatePosition = passHysteresis && lastTarget.isValid && teammate == lastTarget.teammate ? lastTarget.bestShiftedTargetOnField : targetOnField;
  targetEvaluations.emplace_back();
  TargetEvaluation& target = targetEvaluations.back();
  target.isValid = true;
  target.ballOnField = ballOnField;
  target.targetOnField = targetOnField;
  target.bestShiftedTargetOnField = targetOnField;
  target.teammate = teammate;
  calculateTeammatePassTarget(target);
  return target;
}

void PassTargetEvaluator::calculateTeammatePassTarget(TargetEvaluation& target) const
{
  const Vector2f& ballPosition = target.ballOnField;
  const Vector2f& teammatePosition = target.targetOnField;
  Rangea& precisionRange = target.precisionRange;
  bool& isAngleFree = target.isAngleFree;

  // During kick-off, the target angle will not be modified because obstacles can be ignored (there can not legally be any opponents in the own half).
  isAngleFree = theGameState.isKickOff() && theGameState.isForOwnTeam();
  // Calculates the angular sectors of the known opponents from the obstacle model
  const std::list<SectorWheel::Sector>& sectors = calculateObstacleSectors(ballPosition, isAngleFree, target.teammate);

  // Determine whether the ball should be ideally passed to the left (=counterclockwise) or right (=clockwise) side of the pass target (from the perspective of the ball), so that the ball ends up between the receiver and the opponent's goal.
  const Geometry::Line ballGoalLine(ballPosition, opponentGoal - ballPosition);
  const float distToLine = Geometry::getDistanceToLineSigned(ballGoalLine, teammatePosition);
  const bool passToLeftSide = distToLine > 0.f;
  const Vector2f targetPosition = (teammatePosition + (opponentGoal - teammatePosition).normalized(passAheadDistance));
  float targetDistance = (targetPosition - ballPosition).norm();
  Angle targetAngle = (targetPosition - ballPosition).angle();

  // When the initial target angle is blocked, it is shifted to a free sector (not blocked by obstacles) in the direction of the opponent's goal first (ideally placing it in front of the receiver).
  Angle modifiedMinAngleOffset = minAngleOffset; // The modifiedMinAngleOffset gets reduced if no free angle can be found.
  bool isIdealSide = false;
  while(!isAngleFree)
  {
    Angle firstSideAngle = targetAngle;
    Angle secondSideAngle = targetAngle;
    // Calculate the closest angles in both directions and pick the one with be best combined target rating. Prefer first angle, because a pass to the goal side of the teammate is more "natural".
    const bool isFirstSideValid = findNextFreeAngle(sectors, firstSideAngle, precisionRange, passToLeftSide, maxAngleOffset, modifiedMinAngleOffset);
    const bool isSecondSideValid = findNextFreeAngle(sectors, secondSideAngle, precisionRange, !passToLeftSide, maxAngleOffsetBehind, modifiedMinAngleOffset);
    isAngleFree = isFirstSideValid || isSecondSideValid;
    if(isFirstSideValid && isSecondSideValid)
    {
      // When both sides of the receiver can be passed to, evaluate the resulting target positions to determine which side would be better.
      const float firstSideRating = getAngleRating(ballPosition, firstSideAngle, true, ballPosition, targetAngle, targetDistance).second;
      const float secondSideRating = getAngleRating(ballPosition, secondSideAngle, false, ballPosition, targetAngle, targetDistance).second;
      if(firstSideRating >= secondSideRating && firstSideRating > 0.f)
      {
        targetAngle = firstSideAngle;
        isIdealSide = true;
      }
      else
      {
        targetAngle = secondSideAngle;
        isIdealSide = false;
      }
    }
    else if(isFirstSideValid)
    {
      targetAngle = firstSideAngle;
      isIdealSide = true;
    }
    else if(isSecondSideValid)
    {
      targetAngle = secondSideAngle;
      isIdealSide = false;
    }
    else if(modifiedMinAngleOffset < 1_deg)
    {
      break;
    }
    modifiedMinAngleOffset -= 1_deg;
  }

  std::tie(target.passRating, target.combinedRating) = getAngleRating(ballPosition, targetAngle, isIdealSide, ballPosition, targetAngle, targetDistance);
  target.bestShiftedTargetOnField = Vector2f::polar(targetDistance, targetAngle) + ballPosition;
};

bool PassTargetEvaluator::findNextFreeAngle(const std::list<SectorWheel::Sector>& sectors, Angle& targetAngle, Rangea& precisionRange, const bool ccw, const Angle maxAngleDeviation, const Angle& modifiedMinAngleOffset) const
{
  const Angle originalTarget = targetAngle;
  const auto handleFreeSector = [&](const SectorWheel::Sector& sector) -> bool
  {
    if(!sector.angleRange.isInside(originalTarget))
    {
      if(sector.angleRange.max < sector.angleRange.min)
        targetAngle = sector.angleRange.max.diffAbs(originalTarget) < sector.angleRange.min.diffAbs(originalTarget) ? sector.angleRange.max : sector.angleRange.min;
      else
        targetAngle = sector.angleRange.limit(originalTarget);
    }
    else
      targetAngle = originalTarget;
    // Corrects the initial angle if it is too close to either edge of the free angle range (i.e. too close to the obstacle to successfully pass by).
    const bool tooCloseMaxBorder = targetAngle.diffAbs(sector.angleRange.max) < modifiedMinAngleOffset;
    const bool tooCloseMinBorder = targetAngle.diffAbs(sector.angleRange.min) < modifiedMinAngleOffset;
    Angle correctedAngle = targetAngle;
    // Shifts the angle only in the current search direction.
    if(tooCloseMaxBorder && !ccw)
      correctedAngle = getNextAngleInRange(sector.angleRange, !ccw, modifiedMinAngleOffset);
    if(tooCloseMinBorder && ccw)
      correctedAngle = getNextAngleInRange(sector.angleRange, ccw, modifiedMinAngleOffset);
    if(setAnglePrecisionRange(targetAngle, precisionRange, correctedAngle, maxAngleDeviation, sector.angleRange))
      return true;
    return false;
  };

  for(auto it = sectors.begin(); it != sectors.end(); ++it)
  {
    // Starts with the sector that contains the initial angle.
    if(!it->angleRange.isInside(targetAngle))
      continue;
    if(it->type == SectorWheel::Sector::free)
    {
      if(handleFreeSector(*it))
        return true;
    }
    auto origIt = it;
    // Iterates over the sectors in the (counter)clockwise direction to find the first free and valid sector.
    do
    {
      if(ccw)
        ++it;
      if(it == (ccw ? sectors.end() : sectors.begin()))
        it = ccw ? sectors.begin() : sectors.end();
      if(!ccw)
        --it;
      if(it->type == SectorWheel::Sector::free && handleFreeSector(*it))
        return true;
    }
    while(it != origIt);
    break;
  }
  return false;
};

// Calculates the angle inside a range that is closest in the (counter)clockwise direction. If the angle range is too small, its center is used.
Angle PassTargetEvaluator::getNextAngleInRange(const Rangea angleRange, const bool ccw, const Angle& modifiedMinAngleOffset) const
{
  const Angle halfSizeOfRange = angleRange.getSize() / 2.f;
  const Angle angleOffset = std::min(modifiedMinAngleOffset, halfSizeOfRange);
  return ccw ?
         angleRange.min + angleOffset :
         angleRange.max - angleOffset;
};

// Calculates the angular sectors of the known opponents from the obstacle model.
std::list<SectorWheel::Sector> PassTargetEvaluator::calculateObstacleSectors(const Vector2f& ballPositionOnField, const bool isAngleFree, const int teammate) const
{
  SectorWheel sectorWheel;
  const float radius = isAngleFree || (theGameState.isFreeKick() && theGameState.isForOwnTeam()) ? 2.f : 4.f;
  sectorWheel.begin(ballPositionOnField);

  const auto addObstacle = [&](const Vector2f& position, const float width)
  {
    if(position.x() > (theFieldDimensions.xPosOpponentGoalLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
      return;
    const float obstacleWidth = width + radius * theBallSpecification.radius + opponentExtraWidth;
    const float obstacleDistance = std::sqrt(std::max((position - ballPositionOnField).squaredNorm() - sqr(obstacleWidth / 2.f), 1.f));
    if(obstacleDistance < theBallSpecification.radius)
      return;
    const float obstacleRadius = std::atan(obstacleWidth / (2.f * obstacleDistance));
    const Angle obstacleDirection = (position - ballPositionOnField).angle();
    sectorWheel.addSector(Rangea(Angle::normalize(obstacleDirection - obstacleRadius), Angle::normalize(obstacleDirection + obstacleRadius)), obstacleDistance, SectorWheel::Sector::obstacle);
  };

  for(const auto& obstacle : theGlobalOpponentsModel.opponents)
    addObstacle(obstacle.position, theRobotDimensions.robotDepth * 2.f);
  for(const auto& mate : theGlobalTeammatesModel.teammates)
  {
    if(mate.playerNumber == teammate) // skip pass target
      continue;
    addObstacle(mate.pose.translation, 200.f);
  }
  return sectorWheel.finish();
};

// Calculates the rating for a target position based on the rating functions for passing and shooting at the opponent's goal.
std::pair<float, float> PassTargetEvaluator::getAngleRating(const Vector2f& ballPosition, const Angle candidateAngle, const bool isIdealSide, const Vector2f basePosition, const Angle targetAngle, const float targetDistance) const
{
  const Vector2f targetPosition = basePosition + Vector2f::polar(targetDistance, candidateAngle);
  // Rating of deviation of new angle to the original angle.
  const Angle maxAngleDeviation = isIdealSide ? maxAngleOffset : maxAngleOffsetBehind;
  const float offsetRating = mapToRange(targetAngle.diffAbs(candidateAngle), 0_deg, maxAngleDeviation, isIdealSide ? Angle(1.f) : Angle(0.6f), 0_deg);
  const float positionRating = thePassEvaluation.getRating(ballPosition, targetPosition, false);
  const float goalRating = theExpectedGoals.getRating(targetPosition, false, true, false);
  const float combinedRating = offsetRating * positionRating * goalRating;
  COMPLEX_DRAWING("option:PassToTeammate:evaluation")
  {
    const Vector2f& ballPositionOnField = ballPosition;
    LINE("option:PassToTeammate:evaluation", ballPositionOnField.x(), ballPositionOnField.y(), targetPosition.x(), targetPosition.y(), 20, Drawings::PenStyle::solidPen, ColorRGBA::violet);
    DRAW_TEXT("option:PassToTeammate:evaluation", targetPosition.x(), targetPosition.y(), 200, ColorRGBA::violet, "p: " << positionRating);
    DRAW_TEXT("option:PassToTeammate:evaluation", targetPosition.x(), targetPosition.y() - 250, 200, ColorRGBA::violet, "g: " << goalRating);
    DRAW_TEXT("option:PassToTeammate:evaluation", targetPosition.x(), targetPosition.y() - 500, 200, ColorRGBA::violet, "o: " << offsetRating);
    DRAW_TEXT("option:PassToTeammate:evaluation", targetPosition.x(), targetPosition.y() - 750, 200, ColorRGBA::violet, "c: " << combinedRating);
  }
  return { positionRating, combinedRating };
};

// Sets the target angle, when it is valid (i.e. within the given parameters).
bool PassTargetEvaluator::setAnglePrecisionRange(Angle& targetAngle, Rangea& precisionRange, const Angle currentAngle, const Angle maxAngleDeviation, const Rangea& angleRange) const
{
  /** No final kick angle will be set, when one of the following conditions is met:
      - the free angle range is too small
      - the angle is too far from the original angle
      - the angle is outside of the free angle range
   */
  const Angle halfSizeOfRange = angleRange.getSize() / 2.f;
  if(halfSizeOfRange < minAngleOffset ||
     targetAngle.diffAbs(currentAngle) > maxAngleDeviation ||
     !angleRange.isInside(currentAngle))
    return false;
  targetAngle = currentAngle;

  precisionRange = angleRange;
  Rangea borderToMinMax(precisionRange.min + maxPrecisionOffset,
                        precisionRange.max - maxPrecisionOffset);
  if(precisionRange.max < precisionRange.min)
  {
    if(targetAngle > precisionRange.min)
      borderToMinMax.max = 180_deg;
    else
      borderToMinMax.min = -180_deg;
  }
  borderToMinMax = Rangea(std::min(targetAngle, borderToMinMax.min),
                          std::max(targetAngle, borderToMinMax.max));
  precisionRange.max = std::max(0.f, borderToMinMax.limit(targetAngle + maxPrecisionOffset) - targetAngle);
  precisionRange.min = std::min(0.f, borderToMinMax.limit(targetAngle - maxPrecisionOffset) - targetAngle);
  return true;
};
