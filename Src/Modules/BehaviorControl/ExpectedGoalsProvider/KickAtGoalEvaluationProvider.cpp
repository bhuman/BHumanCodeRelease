/**
 * @file KickAtGoalEvaluationProvider.cpp
 *
 * This file defines a representation that evaluates a target position for passes.
 *
 * @author Philip Reichenberg
 */

#include "KickAtGoalEvaluationProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Tools/Modeling/Obstacle.h"

MAKE_MODULE(KickAtGoalEvaluationProvider);

KickAtGoalEvaluationProvider::KickAtGoalEvaluationProvider() :
  leftGoalPost(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal),
  rightGoalPost(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal),
  goalLine(Vector2f(theFieldDimensions.xPosOpponentGoalLine + theFieldDimensions.fieldLinesWidth * 0.5f + theBallSpecification.radius, 0.f), Vector2f(0.f, 1.f))
{
}

void KickAtGoalEvaluationProvider::update(KickAtGoalEvaluation& theKickAtGoalEvaluation)
{
  wasValidLastFrame = theKickAtGoalEvaluation.kickInfo.isValid;
  theKickAtGoalEvaluation.kickInfo.isValid = false;

  if(!wasUpdatedLastFrame)
  {
    goalLineIntersection = Vector2f::Zero();
    theKickAtGoalEvaluation.kickInfo.isValid = false;
    wasValidLastFrame = false;
    aimingAtGoal = false;
    wasUpdatedLastFrame = false;
  }

  wasUpdatedLastFrame = false;

  theKickAtGoalEvaluation.update = [&]()
  {
    if(wasUpdatedLastFrame)
      return;

    wasUpdatedLastFrame = true;
    const bool kickAtGoalDisabled = (theGameState.isKickIn() && theGameState.isForOwnTeam());

    if(!wasValidLastFrame || kickAtGoalDisabled)
    {
      goalLineIntersection = Vector2f::Zero();
      theKickAtGoalEvaluation.kickInfo.isValid = false;
      wasValidLastFrame = false;
      aimingAtGoal = false;
    }

    if(kickAtGoalDisabled)
      return;

    const Angle lastTargetAngleOnField = (goalLineIntersection - theFieldBall.positionOnField).angle();
    const bool lastAimingAtGoal = aimingAtGoal;
    aimingAtGoal = false;

    const float minBallGoalPostOffset = theFieldDimensions.goalPostRadius + theBallSpecification.radius;
    const Angle leftAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (leftGoalPost - theFieldBall.positionOnField).norm()));
    const Angle rightAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (rightGoalPost - theFieldBall.positionOnField).norm()));
    const Angle angleToLeftPost = (leftGoalPost - theFieldBall.positionOnField).angle() - leftAngleOffset;
    const Angle angleToRightPost = (rightGoalPost - theFieldBall.positionOnField).angle() + rightAngleOffset;

    RAY("option:KickAtGoal:goalSector", theFieldBall.positionOnField, angleToLeftPost, 10, Drawings::solidPen, ColorRGBA::blue);
    RAY("option:KickAtGoal:goalSector", theFieldBall.positionOnField, angleToRightPost, 10, Drawings::solidPen, ColorRGBA::red);

    Angle kickDirection = 0_deg;
    const Angle mostOptimalGoalDirectionInField = (Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f) - theFieldBall.positionOnField).angle();
    std::list<SectorWheel::Sector> sectorsOriginal;

    if(theFieldBall.positionOnField.x() > theFieldDimensions.xPosOpponentGoalPost - minBallGoalPostOffset - (lastAimingAtGoal ? hysteresisNumber : 0.f) &&
       std::abs(theFieldBall.positionOnField.y()) < theFieldDimensions.yPosLeftGoal - minBallGoalPostOffset - (lastAimingAtGoal ? 0.f : hysteresisNumber))
    {
      const Rangea clampedRange(std::max<Angle>(angleToRightPost, -pi_2), std::min<Angle>(angleToLeftPost, pi_2));
      const Rangea searchRange = clampedRange.getSize() > 2.f * kickInaccuracy ?
                                 Rangea(clampedRange.min + kickInaccuracy, clampedRange.max - kickInaccuracy) :
                                 Rangea(clampedRange.getCenter());

      kickDirection = searchRange.getCenter() - theRobotPose.rotation;

      theKickAtGoalEvaluation.kickInfo.isValid = true;
      VERIFY(Geometry::getIntersectionOfLines(goalLine,
                                              Geometry::Line(Pose2f(Angle::normalize(theRobotPose.rotation + theKickAtGoalEvaluation.kickInfo.kickDirection),
                                                             theFieldBall.positionOnField)), goalLineIntersection));
      CROSS("option:KickAtGoal:goalLineIntersection", goalLineIntersection.x(), goalLineIntersection.y(), 100, 20, Drawings::solidPen, ColorRGBA::white);
      aimingAtGoal = true;
    }
    else if(angleToLeftPost < angleToRightPost ||
            (theFieldBall.positionOnField.x() >= theFieldDimensions.xPosOpponentGoalPost - minBallGoalPostOffset - (lastAimingAtGoal ? 0.f : hysteresisNumber) &&
             std::abs(theFieldBall.positionOnField.y()) >= theFieldDimensions.yPosLeftGoal - minBallGoalPostOffset - (lastAimingAtGoal ? 0.f : hysteresisNumber)))
    {
      aimingAtGoal = false;
    }
    else
    {
      ASSERT(std::abs(angleToLeftPost) <= pi_2);
      ASSERT(std::abs(angleToRightPost) <= pi_2);

      // Helper function
      const auto addObstacle = [&](std::vector<ObstacleSector>& obstacleSectors, const Obstacle& obstacle)
      {
        const Vector2f obstacleOnField = theRobotPose * obstacle.center;
        if(obstacleOnField.x() > (theFieldDimensions.xPosOpponentGoalLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
          return;

        const float widthScale = 1.f - Rangef::ZeroOneRange().limit((theFrameInfo.getTimeSince(obstacle.lastSeen) - 1000) / 4000.f);
        const float width = widthScale * ((obstacle.left - obstacle.right).norm() + 2.5f * theBallSpecification.radius);
        const float distance = std::sqrt(std::max((obstacleOnField - theFieldBall.positionOnField).squaredNorm() - sqr(width / 2.f), 1.f));
        if(distance < theBallSpecification.radius)
          return;

        const float radius = std::atan(width / (2.f * distance));
        const Angle direction = (obstacleOnField - theFieldBall.positionOnField).angle();
        obstacleSectors.emplace_back();
        obstacleSectors.back().sector = Rangea(Angle::normalize(direction - radius), Angle::normalize(direction + radius));
        obstacleSectors.back().distance = distance;
        obstacleSectors.back().x = obstacleOnField.x();
        // If the previous target is inside this sector, artificially increase its x coordinate so it will potentially be culled before other sectors.
        if(lastAimingAtGoal && obstacleSectors.back().sector.isInside(lastTargetAngleOnField))
          obstacleSectors.back().x += hysteresisNumber;
      };

      // Prepare obstacle sectors.
      std::vector<ObstacleSector> obstacleSectors;

      // Add obstacles we saw
      for(const Obstacle& obstacle : theObstacleModel.obstacles)
        addObstacle(obstacleSectors, obstacle);

      // Sort obstacles according to their absolute x coordinate (to ease culling later on).
      std::sort(obstacleSectors.begin(), obstacleSectors.end(), [](const ObstacleSector& s1, const ObstacleSector& s2) { return s1.x < s2.x; });

      // Determine the x coordinate up to which obstacles may be culled.
      const float cullFactor = std::max(0.f, std::min(theFieldBall.positionOnField.x() / theFieldDimensions.xPosOpponentPenaltyMark, 1.f));
      const float cullBeyondX = cullFactor * theFieldDimensions.xPosOpponentGoalLine +
                                (1.f - cullFactor) * theFieldDimensions.xPosOpponentPenaltyMark;
      LINE("option:KickAtGoal:cullLine", cullBeyondX, theFieldDimensions.yPosLeftTouchline, cullBeyondX, theFieldDimensions.yPosRightTouchline, 10, Drawings::solidPen, ColorRGBA::black);

      SectorWheel wheel;
      std::list<SectorWheel::Sector> sectors;
      bool isLargeEnough = false;
      do
      {
        wheel.begin(theFieldBall.positionOnField);
        wheel.addSector(Rangea(angleToRightPost, angleToLeftPost), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);
        for(const ObstacleSector& obstacleSector : obstacleSectors)
          wheel.addSector(obstacleSector.sector, obstacleSector.distance, SectorWheel::Sector::obstacle);
        sectors = wheel.finish();

        for(const SectorWheel::Sector& sector : sectors)
          if(sector.type == SectorWheel::Sector::goal &&
             sector.angleRange.getSize() >= ((lastAimingAtGoal && sector.angleRange.isInside(lastTargetAngleOnField)) ? Angle(minOpeningAngle * 0.25f) : minOpeningAngle))
          {
            isLargeEnough = true;
            break;
          }
      }
      while(!isLargeEnough && !obstacleSectors.empty() && obstacleSectors.back().x > cullBeyondX && (obstacleSectors.pop_back(), true));

      DRAW_SECTOR_WHEEL("option:KickAtGoal:wheel", sectors, theFieldBall.positionOnField);
      sectorsOriginal = sectors;

      sectors.erase(std::remove_if(sectors.begin(), sectors.end(), [](const SectorWheel::Sector& sector) { return sector.type != SectorWheel::Sector::goal; }), sectors.end());
      // We can be sure that the goal angles are normalized and between -pi/2 and pi/2. Therefore, the angles do not need special handling.
      sectors.sort([&](const SectorWheel::Sector& sector1, const SectorWheel::Sector& sector2) -> bool
      {
        const float openingAngle1 = sector1.angleRange.getSize() + ((lastAimingAtGoal && sector1.angleRange.isInside(lastTargetAngleOnField)) ? hysteresisAngle : 0_deg);
        const float openingAngle2 = sector2.angleRange.getSize() + ((lastAimingAtGoal && sector2.angleRange.isInside(lastTargetAngleOnField)) ? hysteresisAngle : 0_deg);
        return openingAngle1 > openingAngle2;
      });

      for(const SectorWheel::Sector& sector : sectors)
      {
        // TODO can't this check be removed?
        const Angle wholeOpeningAngle = sector.angleRange.getSize();
        if(wholeOpeningAngle < minOpeningAngle * (lastAimingAtGoal && sector.angleRange.isInside(lastTargetAngleOnField) ? 0.25f : 1.f))
          continue;

        if(!ignoreRangeLimit)
        {
          const float usedRange = theKickInfo[useKick].range.max + (wasValidLastFrame ? hysteresisKickRangeExtension : 0.f);
          if(usedRange < (goalLine.base.x() - theFieldBall.positionOnField.x()))
            continue;

          const Angle absAngle = std::acos((goalLine.base.x() - theFieldBall.positionOnField.x()) / usedRange);
          ASSERT(!std::isnan(float(absAngle)));
          ASSERT(absAngle <= pi_2);
          if(sector.angleRange.max < -absAngle || sector.angleRange.min > absAngle)
            continue;

          const Rangea clampedRange(std::max(sector.angleRange.min, -absAngle), std::min(sector.angleRange.max, absAngle));
          ASSERT(clampedRange.getSize() <= sector.angleRange.getSize());
          const Angle openingAngle = clampedRange.getSize() + ((lastAimingAtGoal && clampedRange.isInside(lastTargetAngleOnField)) ? minOpeningAngle * 0.5f : 0.f);
          if(openingAngle < minOpeningAngle)
            continue;

          const Rangea searchRange = clampedRange.getSize() > 2.f * kickInaccuracy ?
                                     Rangea(clampedRange.min + kickInaccuracy, clampedRange.max - kickInaccuracy) :
                                     Rangea(clampedRange.getCenter());
          Angle kickAngleInField = searchRange.limit(mostOptimalGoalDirectionInField);
          if(theGameState.isFreeKick())
            kickAngleInField = (kickAngleInField + searchRange.getCenter()) / 2.f;
          kickDirection = kickAngleInField - theRobotPose.rotation;
          theKickAtGoalEvaluation.kickInfo.isValid = true;
          break;
        }
        else
        {
          const Angle sectorShiftValue = sector.angleRange.getSize() * sectorShift;
          const Rangea searchRange(sector.angleRange.min + sectorShiftValue, sector.angleRange.max - sectorShiftValue);
          Angle kickAngleInField = searchRange.limit(mostOptimalGoalDirectionInField);
          if(theGameState.isFreeKick())
            kickAngleInField = (kickAngleInField + searchRange.getCenter()) / 2.f;
          kickDirection = kickAngleInField - theRobotPose.rotation;
          theKickAtGoalEvaluation.kickInfo.isValid = true;
          break;
        }
      }
    }

    if(theKickAtGoalEvaluation.kickInfo.isValid)
    {
      aimingAtGoal = true;
      theKickAtGoalEvaluation.kickInfo.kickDirection = Angle::normalize(kickDirection);
      theKickAtGoalEvaluation.kickInfo.kickType = getDistanceToNearestObstacle(theKickAtGoalEvaluation, sectorsOriginal) < (theKickAtGoalEvaluation.kickInfo.kickType == useKick ? distanceToObstacle.min : distanceToObstacle.max) ? replaceKick : useKick;
      VERIFY(Geometry::getIntersectionOfLines(goalLine,
                                              Geometry::Line(Pose2f(Angle::normalize(theRobotPose.rotation + theKickAtGoalEvaluation.kickInfo.kickDirection),
                                                             theFieldBall.positionOnField)), goalLineIntersection));
      CROSS("option:KickAtGoal:goalLineIntersection", goalLineIntersection.x(), goalLineIntersection.y(), 100, 20, Drawings::solidPen, ColorRGBA::white);
    }
    else
      theKickAtGoalEvaluation.kickInfo.kickType = useKick;
  };
}

Angle KickAtGoalEvaluationProvider::getDistanceToNearestObstacle(const KickAtGoalEvaluation& theKickAtGoalEvaluation, const std::list<SectorWheel::Sector>& sectors)
{
  if(sectors.empty())
    return std::numeric_limits<Angle>::max();

  auto extraObstacleDiff = [&](const float distance, const Angle direction) -> Angle
  {
    return mapToRange((theFieldBall.positionOnField + Vector2f::polar(distance, direction)).x(), (theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnPenaltyArea) * 0.5f, theFieldDimensions.xPosOwnPenaltyArea + 1000.f, 0.f, static_cast<float>(distanceToObstacle.max));
  };

  auto getClosestObstacleAngle = [&](const std::list<SectorWheel::Sector>& sectors, const std::list<SectorWheel::Sector>::const_iterator & it, const Angle directionInField, bool isLeftSearch)
  {
    auto itOther = it;
    if(isLeftSearch)
    {
      while(true)
      {
        itOther++;
        if(itOther == sectors.end())
          itOther = sectors.begin();
        if(itOther == it)
          return directionInField + 100_deg;
        if(itOther->type == SectorWheel::Sector::obstacle)
          return directionInField.diffAbs(itOther->angleRange.min) + extraObstacleDiff(itOther->distance, itOther->angleRange.getCenter());
      }
    }
    else
    {
      while(true)
      {
        if(itOther == sectors.begin())
          itOther = sectors.end();
        itOther--;
        if(itOther == it)
          return directionInField + 100_deg;
        if(itOther->type == SectorWheel::Sector::obstacle)
          return directionInField.diffAbs(itOther->angleRange.max) + extraObstacleDiff(itOther->distance, itOther->angleRange.getCenter());
      }
    }
  };

  const Angle directionInField = theKickAtGoalEvaluation.kickInfo.kickDirection + theRobotPose.rotation;
  for(std::list<SectorWheel::Sector>::const_iterator it = sectors.begin(); it != sectors.end(); it++)
  {
    if(it->angleRange.isInside(directionInField))
      return std::min(getClosestObstacleAngle(sectors, it, directionInField, true), getClosestObstacleAngle(sectors, it, directionInField, false));
  }
  return std::numeric_limits<Angle>::max();
}
