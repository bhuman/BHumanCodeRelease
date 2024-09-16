/**
 * @file DribbleTargetProvider.cpp
 *
 * This file implements a module that calculates the ball position after the execution of the DribbleToGoal-Skill.
 *
 * @author Nico Holsten
 */

#include "DribbleTargetProvider.h"
#include <map>

MAKE_MODULE(DribbleTargetProvider);

DribbleTargetProvider::DribbleTargetProvider()
{
  lastDribbleAngle = 0_deg;
  kickAngles = std::list<SectorWheel::Sector>();
}

void DribbleTargetProvider::update(DribbleTarget& theDribbleTarget)
{
  checkedDribbleTargets.clear();

  theDribbleTarget.calculateDribbleAngle = [this](const Vector2f& ballPosition) -> Angle
  {
    for(const auto& checkedTargets : checkedDribbleTargets)
    {
      if(checkedTargets.startFieldPosition == ballPosition)
        return checkedTargets.dribbleAngle;
    }

    const Angle direction = calculateDribbleAngle(ballPosition);
    checkedDribbleTargets.emplace_back(ballPosition, ballPosition + Vector2f::polar(dribbleRange, direction), direction);
    return direction;
  };
  theDribbleTarget.getTarget = [this](const Vector2f& ballPosition) -> Vector2f
  {
    for(const auto& checkedTargets : checkedDribbleTargets)
    {
      if(checkedTargets.startFieldPosition == ballPosition)
        return checkedTargets.endFieldPosition;
    }

    const Angle direction = calculateDribbleAngle(ballPosition);
    checkedDribbleTargets.emplace_back(ballPosition, ballPosition + Vector2f::polar(dribbleRange, direction), direction);
    return checkedDribbleTargets.back().endFieldPosition;
  };
}

struct ObstacleSector
{
  Obstacle::Type type;
  Rangea sector; /**< The angular range relative to the ball that the obstacle blocks. */
  float distance; /**< The distance of the obstacle to the ball. */
  float x; /**< The x coordinate on field of the obstacle. */
};

float DribbleTargetProvider::getDistanceToFieldBorder(const Vector2f& startPosition, const Angle& direction)
{
  float distanceToFieldBorderSquared = std::numeric_limits<float>::max();
  Vector2f i1(0.f, 0.f);
  Vector2f i2(0.f, 0.f);
  static_cast<void>(Geometry::getIntersectionPointsOfLineAndRectangle(bottomRightField, topLeftField,
                    Geometry::Line(startPosition, Vector2f::polar(1.f, direction)),
                    i1, i2));
  Vector2f ballToIntersection1 = i1 - startPosition;
  Vector2f ballToIntersection2 = i2 - startPosition;
  // the 5_degs is just a arbitrary number, because the angle to the intersection point is only for a small fraction different to the original kickAngle, resulting from rounding errors.
  if(std::abs(ballToIntersection1.angle() - direction) < 5_deg)
    distanceToFieldBorderSquared = ballToIntersection1.squaredNorm();
  else if(std::abs(ballToIntersection2.angle() - direction) < 5_deg)
    distanceToFieldBorderSquared = ballToIntersection2.squaredNorm();
  return distanceToFieldBorderSquared;
};

void DribbleTargetProvider::calculateSectorWheel()
{
  std::vector<ObstacleSector> obstacleSectors;
  for(const Obstacle& obstacle : theObstacleModel.obstacles)
  {
    const Vector2f obstacleOnField = theRobotPose * obstacle.center;
    if(obstacleOnField.x() > (theFieldDimensions.xPosOpponentGoalLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
      continue;

    const float ballFactor = 3.f;

    const float width = (obstacle.left - obstacle.right).norm() + ballFactor * theBallSpecification.radius;
    const float distance = (obstacleOnField - theFieldBall.positionOnField).norm();

    const float radius = std::atan(width / (2.f * distance));
    const Angle direction = (obstacleOnField - theFieldBall.positionOnField).angle();
    obstacleSectors.emplace_back();
    Rangea obstacleSectorRange = Rangea(Angle::normalize(direction - radius), Angle::normalize(direction + radius));
    obstacleSectors.back().sector = obstacleSectorRange;
    obstacleSectors.back().distance = distance;
    obstacleSectors.back().x = obstacleOnField.x();
    obstacleSectors.back().type = Obstacle::opponent;
  }

  SectorWheel wheel;
  std::list<SectorWheel::Sector> sectors;
  wheel.begin(theFieldBall.positionOnField);

  const float minBallGoalPostOffsetSquared = theFieldDimensions.goalPostRadius + theBallSpecification.radius;
  const Angle leftAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffsetSquared / (leftGoalPost - theFieldBall.positionOnField).norm()));
  const Angle rightAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffsetSquared / (rightGoalPost - theFieldBall.positionOnField).norm()));
  const Angle angleToLeftPost = Angle::normalize((leftGoalPost - theFieldBall.positionOnField).angle() - leftAngleOffset);
  const Angle angleToRightPost = Angle::normalize((rightGoalPost - theFieldBall.positionOnField).angle() + rightAngleOffset);

  if(angleToLeftPost > angleToRightPost)
    wheel.addSector(Rangea(angleToRightPost, angleToLeftPost), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);

  for(const ObstacleSector& obstacleSector : obstacleSectors)
    wheel.addSector(obstacleSector.sector, obstacleSector.distance, SectorWheel::Sector::obstacle);

  kickAngles = wheel.finish();

  SectorWheel wheelFiltered;
  wheelFiltered.begin(theFieldBall.positionOnField);
  SectorWheel::Sector* lastSector = nullptr;
  for(SectorWheel::Sector& sector : kickAngles)
  {
    if(lastSector == nullptr)
      lastSector = &sector;
    else
    {
      const bool mergeToPreviousObstacle = lastSector->distance < obstacleDistance && (sector.distance < obstacleDistance || (sector.angleRange.getSize() < minFreeSector && sector.angleRange.min < sector.angleRange.max));
      const bool mergeCurrentObstalceWithPreviousSmallSector = lastSector->angleRange.getSize() < minFreeSector && lastSector->angleRange.min < lastSector->angleRange.max && sector.distance < obstacleDistance;
      if((mergeToPreviousObstacle || mergeCurrentObstalceWithPreviousSmallSector) &&
         sector.type != SectorWheel::Sector::teammate && sector.type != SectorWheel::Sector::goal &&
         lastSector->type != SectorWheel::Sector::teammate && lastSector->type != SectorWheel::Sector::goal)
      {
        lastSector->angleRange.max = sector.angleRange.max;
        if(!mergeToPreviousObstacle && mergeCurrentObstalceWithPreviousSmallSector)
        {
          lastSector->type = sector.type;
          lastSector->distance = sector.distance;
        }
      }
      else
      {
        wheelFiltered.addSector(lastSector->angleRange, lastSector->distance, lastSector->type);
        lastSector = &sector;
      }
    }
  }

  if(lastSector != nullptr)
  {
    wheelFiltered.addSector(lastSector->angleRange, lastSector->distance, lastSector->type);
  }
  kickAngles = wheelFiltered.finish();

  COMPLEX_DRAWING("option:DribbleTargetProvider:wheel")
  {
    DRAW_SECTOR_WHEEL("option:DribbleTargetProvider:wheel", kickAngles, theFieldBall.positionOnField);
  }
}

Angle DribbleTargetProvider::calculateDribbleAngle(Vector2f position)
{
  PotentialValue pv;
  // clip ball inside the field
  xMinMax.clamp(position.x());
  yMinMax.clamp(position.y());
  const Vector2f startPosition = position - position.normalized(0.001f); // To ensure we are not on the field line
  const Vector2f basisChange = Vector2f::polar(100.f, lastDribbleAngle);
  COMPLEX_DRAWING("option:DribbleTargetProvider:step")
  {
    const Vector2f basisChangeNormalized = basisChange.normalized(stepLength);
    ARROW("option:DribbleTargetProvider:step", position.x(), position.y(), position.x() + searchStepDrawScale * basisChangeNormalized.x(), position.y() + searchStepDrawScale * basisChangeNormalized.y(), searchStepDrawWidth, Drawings::solidPen, ColorRGBA::red);
  }
  position += basisChange;
  // search iterative for the dribble direction
  for(int i = 0; i < iterationSteps; i++)
  {
    pv = theFieldRating.potentialFieldOnly(position.x(), position.y(), true);
    theFieldRating.getObstaclePotential(pv, position.x(), position.y(), true);
    const float directionNorm = pv.direction.norm();
    if(directionNorm == 0.f)
      break;
    Vector2f direction = pv.direction / directionNorm;
    direction *= stepLength;

    COMPLEX_DRAWING("option:DribbleTargetProvider:step")
    {
      if(i % searchStepDrawModulo == 0 && i != 0) // skip first and only draw every 5th
        ARROW("option:DribbleTargetProvider:step", position.x(), position.y(), position.x() + searchStepDrawScale * direction.x(), position.y() + searchStepDrawScale * direction.y(), searchStepDrawWidth, Drawings::solidPen, checkedDribbleTargets.size() == 0 ? ColorRGBA::black : ColorRGBA::cyan);
    }

    position += direction;
    if(position.x() > theFieldDimensions.xPosOpponentGoalLine && position.y() < theFieldDimensions.yPosLeftGoal && position.y() > theFieldDimensions.yPosRightGoal)
      break;
  }

  // get dribble angle
  Angle dribbleAngleInField = (position - startPosition).angle();
  calculateSectorWheel();

  COMPLEX_DRAWING("option:DribbleTargetProvider:direction")
  {
    const Vector2f pos2 = theFieldInterceptBall.interceptedEndPositionOnField + Vector2f::polar(300.f, dribbleAngleInField);
    ARROW("option:DribbleTargetProvider:direction", theFieldInterceptBall.interceptedEndPositionOnField.x(), theFieldInterceptBall.interceptedEndPositionOnField.y(), pos2.x(), pos2.y(), searchStepDrawWidth, Drawings::solidPen, ColorRGBA::cyan);
  }

  const float distanceToFieldBorderThresholdSquared = sqr(dribbleRange + safeDistanceToFieldBoarder);
  // check if we would dribble towards an obstacle and try to avoid it, without dribbling outside of the field
  for(auto it = kickAngles.begin(); it != kickAngles.end(); it++)
  {
    if(it->angleRange.isInside(dribbleAngleInField))
    {
      // obstacle is far away, so just dribble in the desired direction
      if(it->distance > obstacleDistance && it->type != SectorWheel::Sector::free)
        break;
      else
      {
        // 1. Free sector, check if the end position would be outside of the field
        if(it->type == SectorWheel::Sector::free && getDistanceToFieldBorder(startPosition, dribbleAngleInField) > distanceToFieldBorderThresholdSquared)
        {
          break;
        }

        // 2. check left and right directions of the sector and find out, which direction is better
        Angle distToMin = dribbleAngleInField - it->angleRange.min;
        Angle distToMax = it->angleRange.max - dribbleAngleInField;

        // 2.1 check for next sector which is closer to the current kick direction
        const Angle& checkDribbleAngle = std::abs(distToMin) < std::abs(distToMax) ? it->angleRange.min - 0.1_deg : it->angleRange.max + 0.1_deg;

        bool isBreak = false;
        // 2.3 check if new end position is outside of the field or inside goal sector
        if(getDistanceToFieldBorder(startPosition, checkDribbleAngle) > distanceToFieldBorderThresholdSquared)
        {
          dribbleAngleInField = checkDribbleAngle;
          break;
        }
        else
        {
          for(const SectorWheel::Sector& sector : kickAngles)
            if(sector.angleRange.isInside(checkDribbleAngle) && sector.type == SectorWheel::Sector::goal)
            {
              dribbleAngleInField = checkDribbleAngle;
              isBreak = true;
              break;
            }
        }
        if(isBreak)
          break;

        // 3. Find the best angle of the 3: max from this, min from next non obstacle left, max from previous non obstacle right
        const Angle maxThisSector = std::abs(distToMin) >= std::abs(distToMax) ? it->angleRange.min - 0.1_deg : it->angleRange.max + 0.1_deg;
        auto leftIt = std::next(it);
        if(leftIt == kickAngles.end())
          leftIt = kickAngles.begin();
        auto rightIt = it;
        if(rightIt == kickAngles.begin())
          rightIt = std::prev(kickAngles.end());
        else // decrementing iterators beyond begin() is not allowed
          rightIt = std::prev(it);
        const Angle minNextLeftSector = leftIt->angleRange.max;
        const Angle maxPreviousRightSector = rightIt->angleRange.min;

        // Use std::map to automatically sort the angles by their absolute difference
        // Therefore if the last emplaced Angle has the smallest difference, it will be the first element in the map
        std::map<float, Angle> map;
        map.emplace(std::abs(dribbleAngleInField - maxThisSector), maxThisSector);
        map.emplace(std::abs(dribbleAngleInField - minNextLeftSector), minNextLeftSector);
        map.emplace(std::abs(dribbleAngleInField - maxPreviousRightSector), maxPreviousRightSector);

        for(const auto& angle : map)
        {
          if(getDistanceToFieldBorder(startPosition, angle.second) > distanceToFieldBorderThresholdSquared)
          {
            isBreak = true;
            dribbleAngleInField = angle.second;
            break;
          }
        }

        if(isBreak)
          break;

        // we can not find a good angle, just pass into the inner field
        dribbleAngleInField = startPosition.y() > 0 ? -90_deg : 90_deg;
        break;
      }
    }
  }
  COMPLEX_DRAWING("option:DribbleTargetProvider:direction")
  {
    Vector2f pos2 = theFieldInterceptBall.interceptedEndPositionOnField + Vector2f::polar(300.f, dribbleAngleInField);
    ARROW("option:DribbleTargetProvider:direction", theFieldInterceptBall.interceptedEndPositionOnField.x(), theFieldInterceptBall.interceptedEndPositionOnField.y(), pos2.x(), pos2.y(), searchStepDrawWidth, Drawings::solidPen, ColorRGBA::red);
  }
  lastDribbleAngle = dribbleAngleInField;
  return dribbleAngleInField;
}
