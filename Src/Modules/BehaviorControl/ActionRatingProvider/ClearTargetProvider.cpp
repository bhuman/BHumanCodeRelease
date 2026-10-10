/**
 * @file ClearTargetProvider.cpp
 *
 * This file implements a module that calculates the ball position after the execution of the ClearBall-Skill.
 *
 * @author Nico Holsten
 */

#include "ClearTargetProvider.h"
#include <map>

MAKE_MODULE(ClearTargetProvider);

void ClearTargetProvider::update(ClearTarget& theClearTarget)
{
  theClearTarget.isValid = false;

  theClearTarget.reset = [this, &theClearTarget]() -> void
  {
    lastKickType = KickInfo::numOfKickTypes;
    lastSector = {};
    theClearTarget.isValid = false;
    theClearTarget.kickAngle = 0_deg;
    theClearTarget.kickLength = 20000.f;
    theClearTarget.kickType = KickInfo::numOfKickTypes;
  };

  theClearTarget.update = [this, &theClearTarget]() -> void
  {
    if(!theClearTarget.isValid)
    {
      calcBestKick(theClearTarget);
    }
  };
}

bool ClearTargetProvider::isNotNearLine(const Vector2f targetPosition)
{
  COMPLEX_DRAWING("option:ClearBall:borders")
  {
    LINE("option:ClearBall:borders", theFieldDimensions.xPosOwnGoalLine + minDistanceToXLine, theFieldDimensions.yPosLeftFieldBorder, theFieldDimensions.xPosOwnGoalLine + minDistanceToXLine, theFieldDimensions.yPosRightFieldBorder, 20, Drawings::PenStyle::solidPen, ColorRGBA::black);
    LINE("option:ClearBall:borders", theFieldDimensions.xPosOpponentGoalLine - minDistanceToXLine, theFieldDimensions.yPosLeftFieldBorder, theFieldDimensions.xPosOpponentGoalLine - minDistanceToXLine, theFieldDimensions.yPosRightFieldBorder, 20, Drawings::PenStyle::solidPen, ColorRGBA::red);
    LINE("option:ClearBall:borders", theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftFieldBorder - minDistanceToYLine, theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftFieldBorder - minDistanceToYLine, 20, Drawings::PenStyle::solidPen, ColorRGBA::blue);
    LINE("option:ClearBall:borders", theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightFieldBorder + minDistanceToYLine, theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightFieldBorder + minDistanceToYLine, 20, Drawings::PenStyle::solidPen, ColorRGBA::yellow);
  }
  return targetPosition.x() > std::min(theFieldDimensions.xPosOwnGoalLine + minDistanceToXLine, theFieldBall.positionOnField.x())
         && targetPosition.x() < std::max(theFieldDimensions.xPosOpponentGoalLine - minDistanceToXLine, theFieldBall.positionOnField.x())
         && targetPosition.y() < std::max(theFieldDimensions.yPosLeftFieldBorder - minDistanceToYLine, theFieldBall.positionOnField.y())
         && targetPosition.y() > std::min(theFieldDimensions.yPosRightFieldBorder + minDistanceToYLine, theFieldBall.positionOnField.y());
}

float ClearTargetProvider::getAngleRating(const Angle candidateAngle, const Vector2f basePosition, const float targetDistance,
                                          [[maybe_unused]] int number)
{
  //calculate xG-value for target position
  const Vector2f targetPosition = basePosition + Vector2f::polar(targetDistance, candidateAngle);
  if(!isNotNearLine(targetPosition))
  {
    return -1.f;
  }
  const float goalRating = theExpectedGoals.getRating(targetPosition, false, true, true);

  //calculate field factor of current position
  //the factor is 1 in ownPenaltyArea, 1...0 between ownPenaltyArea and HalfwayLine and 0 in oppositeHalf
  //except when it is a own free kick
  float fieldFactor = mapToRange(theFieldBall.positionOnField.x(), theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosHalfwayLine, 1.f, 0.f);
  if(theGameState.isFreeKick() && theGameState.isForOwnTeam())
  {
    fieldFactor = 1.f;
  }

  const float maxHysteresisBonus = mapToRange(theFieldBall.recentBallPositionRelative().norm(), ballDistanceRatingInterpolation.min, ballDistanceRatingInterpolation.max, lastKickRatingBonus.max, lastKickRatingBonus.min);
  const float bonusRating = lastKickType == KickInfo::numOfKickTypes ? 0.f : mapToRange(std::abs(candidateAngle - bestKickDirection), static_cast<float>(kickDirectionHystersisRange.min), static_cast<float>(kickDirectionHystersisRange.max), maxHysteresisBonus, 0.f);

  COMPLEX_DRAWING("option:ClearBall:evaluation")
  {
    std::string closer = "";
    if(isTeammateCloseToBall(candidateAngle, basePosition, targetDistance))
    {
      closer = "*";
    }
    const Vector2f& ballPositionOnField = theFieldBall.positionOnField;
    LINE("option:ClearBall:evaluation", ballPositionOnField.x(), ballPositionOnField.y(), targetPosition.x(), targetPosition.y(), 20, Drawings::PenStyle::solidPen, ColorRGBA::violet);
    DRAW_TEXT("option:ClearBall:evaluation", targetPosition.x(), targetPosition.y() - 250, 200, ColorRGBA::violet, number << ": " << goalRating * fieldFactor + bonusRating << closer);
  }

  //return xG-value of the target position times field factor of the current position + hystersis
  return goalRating * fieldFactor + bonusRating;
}

bool ClearTargetProvider::isTeammateCloseToBall(const Angle candidateAngle, const Vector2f basePosition, const float targetDistance)
{
  const Vector2f targetPosition = basePosition + Vector2f::polar(targetDistance, candidateAngle);
  float teammateDistance = std::numeric_limits<float>::max();
  float opponentDistance = std::numeric_limits<float>::max();
  for(const auto& teammate : theGlobalTeammatesModel.teammates)
  {
    float distance = (teammate.pose.translation - targetPosition).squaredNorm();
    if(distance < teammateDistance)
    {
      teammateDistance = distance;
    }
  }
  for(const auto& opponent : theGlobalOpponentsModel.opponents)
  {
    float distance = (opponent.position - targetPosition).squaredNorm();
    if(distance < opponentDistance)
    {
      opponentDistance = distance;
    }
  }
  return teammateDistance < opponentDistance;
}

void ClearTargetProvider::createSmallSectors(Angle& var, Angle end, std::list<SectorWheel::Sector>& newSmallSectors, SectorWheel::Sector& newSmallSector)
{
  while(var + smallSectorRange < end)
  {
    newSmallSector.angleRange.min = var;
    newSmallSector.angleRange.max = var + smallSectorRange;
    var += smallSectorRange;
    newSmallSectors.push_back(newSmallSector);
  }
}

void ClearTargetProvider::calcBestKick(ClearTarget& theClearTarget)
{
  // Avoid own goal
  const Vector2f leftGoalPost(std::min(theFieldDimensions.xPosOwnGoalPost, theFieldBall.positionOnField.x() - theFieldDimensions.goalPostRadius), theFieldDimensions.yPosLeftGoal);
  const Vector2f rightGoalPost(std::min(theFieldDimensions.xPosOwnGoalPost, theFieldBall.positionOnField.x() - theFieldDimensions.goalPostRadius), theFieldDimensions.yPosRightGoal);
  const Vector2f ballInLeftGoalPost = theFieldBall.positionOnField - leftGoalPost;
  Vector2f ballInRightGoalPost = theFieldBall.positionOnField - rightGoalPost;
  const float distBallLeftGoalPost = ballInLeftGoalPost.norm();
  const float distBallRightGoalPost = ballInRightGoalPost.norm();
  const float minBallGoalPostDistance = theFieldDimensions.goalPostRadius + theBallSpecification.radius + ballGoalPostTangentOffset;
  const Angle ballLeftGoalPostTangentAngleOffset = std::asin(std::min(1.f, minBallGoalPostDistance / distBallLeftGoalPost));
  const Angle ballRightGoalPostTangentAngleOffset = std::asin(std::min(1.f, minBallGoalPostDistance / distBallRightGoalPost));
  const Angle ballLeftGoalPostTangentAngle = Angle::normalize((-ballInLeftGoalPost).angle() + (-ballLeftGoalPostTangentAngleOffset));
  const Angle ballRightGoalPostTangentAngle = Angle::normalize((-ballInRightGoalPost).angle() + (ballRightGoalPostTangentAngleOffset));
  ASSERT(!std::isnan(float(ballRightGoalPostTangentAngle)));
  ASSERT(!std::isnan(float(ballLeftGoalPostTangentAngle)));

  SectorWheel blueprintWheel;
  blueprintWheel.begin(theFieldBall.positionOnField);
  blueprintWheel.addSector(Rangea(ballLeftGoalPostTangentAngle, ballRightGoalPostTangentAngle), 0.f, SectorWheel::Sector::erased);

  // Avoid opponent goal if scoring is not allowed
  if(!theIndirectKick.allowDirectKick)
  {
    const Vector2f leftGoalPost(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
    const Vector2f rightGoalPost(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
    const float minBallGoalPostOffset = theFieldDimensions.goalPostRadius + theBallSpecification.radius + ballGoalPostTangentOffset;
    const Angle leftAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (leftGoalPost - theFieldBall.positionOnField).norm()));
    const Angle rightAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (rightGoalPost - theFieldBall.positionOnField).norm()));
    const Angle angleToLeftPost = (leftGoalPost - theFieldBall.positionOnField).angle() + leftAngleOffset;
    const Angle angleToRightPost = (rightGoalPost - theFieldBall.positionOnField).angle() - rightAngleOffset;
    ASSERT(!std::isnan(float(angleToLeftPost)));
    ASSERT(!std::isnan(float(angleToRightPost)));
    blueprintWheel.addSector(Rangea(angleToRightPost, angleToLeftPost), 0.f, SectorWheel::Sector::erased);
  }

  for(const Obstacle& obstacle : theObstacleModel.obstacles)
  {
    const Vector2f obstacleOnField = theRobotPose * obstacle.center;

    // There may be a lot of "low-quality" obstacles behind on the border strip.
    if(obstacleOnField.x() < theFieldDimensions.xPosOwnGoalLine + 300.f)
      continue;

    const float width = (obstacle.left - obstacle.right).norm() + 2.f * theBallSpecification.radius + additionalObstacleWidth;
    const float distance = std::sqrt(std::max((obstacleOnField - theFieldBall.positionOnField).squaredNorm() - sqr(width / 2.f), 1.f));
    if(distance < theBallSpecification.radius)
      continue;

    const float radius = std::atan(width / (2.f * distance));
    const Angle direction = (obstacleOnField - theFieldBall.positionOnField).angle();
    const Rangea angleRange(direction - radius, direction + radius);

    if(!angleRange.contains(ballLeftGoalPostTangentAngle) && !angleRange.contains(ballRightGoalPostTangentAngle))
    {
      if(obstacle.isTeammate())
      {
        blueprintWheel.addSector(angleRange, distance, SectorWheel::Sector::teammate);
      }
      else
      {
        blueprintWheel.addSector(angleRange, distance, SectorWheel::Sector::obstacle);
      }
    }
  }

  SectorWheel wheel = blueprintWheel;
  auto sectors = wheel.finish();
  std::list<SectorWheel::Sector> newSmallSectors;
  std::vector<KickInfo::KickType> kickTypes = availableKicks;
  //In Free kicks use other kicks
  if(!extraKicksSetPieces.empty() && theGameState.isFreeKick() && theGameState.isForOwnTeam())
    kickTypes = extraKicksSetPieces;
  for(const SectorWheel::Sector& sector : sectors)
  {
    //divide this large free sector in smaller ones
    if(sector.type == SectorWheel::Sector::free && sector.angleRange.getSize() > largeSector)
    {
      Angle start = sector.angleRange.min;
      Angle end = sector.angleRange.max;
      Angle var = start;
      SectorWheel::Sector newSmallSector;
      newSmallSector.distance = sector.distance;
      newSmallSector.type = SectorWheel::Sector::free;
      //the easier case where min of the large sector is smaller than the max
      if(start <= end)
      {
        createSmallSectors(var, end, newSmallSectors, newSmallSector);
        if(end - var > smallSector)
        {
          newSmallSector.angleRange.min = var;
          newSmallSector.angleRange.max = end;
          newSmallSectors.push_back(newSmallSector);
        }
      }
      //the harder case where max is smaller than min
      else
      {
        //we add values to reach pi
        var = sector.angleRange.min;
        createSmallSectors(var, pi, newSmallSectors, newSmallSector);
        if(!newSmallSectors.empty())
        {
          newSmallSector.angleRange.min = var;
          newSmallSector.angleRange.max = pi;
          newSmallSectors.push_back(newSmallSector);
        }
        //we add values to get away from -pi
        var = -pi;
        createSmallSectors(var, end, newSmallSectors, newSmallSector);
        if(!newSmallSectors.empty())
        {
          newSmallSector.angleRange.min = var;
          newSmallSector.angleRange.max = end;
          newSmallSectors.push_back(newSmallSector);
        }
      }
    }
  }
  DRAW_SECTOR_WHEEL("option:ClearBall:clearBall", sectors, theFieldBall.positionOnField);
  for(const SectorWheel::Sector& item : newSmallSectors)
    sectors.push_back(item);
  sectors.remove_if([](SectorWheel::Sector item) { return item.angleRange.getSize() >= 30_deg; });

  float bestXG = 0.f;//best xG of all sectors and kick types
  float leftTeammateBonus = 0.0f; //when the current sector is obstacle or teammate we add or sub a bonus to the next sector if it is a free one
  bool applyRightTeamMateBonus = false;//when the current sector is free, we let the next sector know, that it has to add or sub a bonus to the current sector if next sector is obstacle/teammate
  float goalRating = 0.f;//xG of current sector and shot type
  KickInfo::KickType lastBestKick = KickInfo::numOfKickTypes; //kick type of last sector
  SectorWheel::Sector lastBestSector; //last sector
  float lastBestKickLength = 20000.f; // last kick length
  Angle lastBestKickDirection;//pose of last sector
  float bestXGOfSector = 0.0f;//best XG of last sector
  std::map<int, float> mapOfRatings;//needed for debug drawings
  COMPLEX_DRAWING("option:ClearBall:zero")
  {
    const Vector2f targetPosition = theFieldBall.positionOnField + Vector2f::polar(10000, -pi);
    const Vector2f& ballPositionOnField = theFieldBall.positionOnField;
    LINE("option:ClearBall:zero", ballPositionOnField.x(), ballPositionOnField.y(), targetPosition.x(), targetPosition.y(), 40, Drawings::PenStyle::dottedPen, ColorRGBA::yellow);
  }
  int i = 0;
  //skip big sectors
  //add the small sectors at the end of the structure
  for(const SectorWheel::Sector& sector : sectors)
  {
    i++;
    if(sector.type == SectorWheel::Sector::free)
    {
      bestXGOfSector = 0.0f;
      //reset teammate bonus since this one is a free sector
      leftTeammateBonus = 0.f;
      applyRightTeamMateBonus = true;

      //sectors with range < 10_deg are too small to pass into
      if(sector.angleRange.getSize() > smallSector)
      {
        const float minKickStrength = minKickDistance.min;// mapToRange(std::abs(sector.angleRange.getCenter()), static_cast<float>(scaleKickDistanceAngleRange.min), static_cast<float>(scaleKickDistanceAngleRange.max), minKickDistance.min, minKickDistance.max);
        //check best kick for the current sector
        for(KickInfo::KickType kickType : kickTypes)
        {
          for(float kickRange = theKickInfo[kickType].range.max; kickRange > minKickStrength && kickRange > theKickInfo[kickType].range.min; kickRange = (kickRange > minKickStrength && kickRange - kickDistanceStepSize < minKickStrength) ? minKickStrength : kickRange - kickDistanceStepSize)
          {
            const Angle center = sector.angleRange.getCenter();
            goalRating = getAngleRating(center, theFieldBall.positionOnField, kickRange, i);
            if(goalRating < 0.f)
            {
              continue;
            }
            if(sector.angleRange.contains(lastSector.angleRange.getCenter()))
              goalRating += hysteresisSector;
            if(kickType == lastKickType)
              goalRating += hysteresisShotType;
            //add/sub bonus of the sector before this one
            goalRating += leftTeammateBonus;
            //save best rating of this sector
            if(goalRating > bestXGOfSector)
            {
              bestXGOfSector = goalRating;
            }
            //check if clearTarget is useful
            if(isTeammateCloseToBall(sector.angleRange.getCenter(), theFieldBall.positionOnField, kickRange))
              goalRating += teammateSummand;
            if(goalRating > bestXG)
            {
              bestXG = goalRating;
              bestSector = sector;
              bestKickDirection = center;
              bestKick = kickType;
              bestKickLength = kickRange;
              lastBestKickLength = kickRange;
              lastBestSector = sector;
              lastBestKickDirection = center;
              lastBestKick = kickType;
            }
          }
        }
      }
      mapOfRatings.emplace(i, bestXGOfSector);
    }
    //add bonus to last sector
    else if(sector.type == SectorWheel::Sector::teammate)
    {
      leftTeammateBonus = teammateSummand;
      if(applyRightTeamMateBonus)
      {
        bestXGOfSector += teammateSummand;
        ASSERT(!mapOfRatings.empty());
        mapOfRatings.erase(std::prev(mapOfRatings.end()));
        mapOfRatings.insert(std::pair(i - 1, bestXGOfSector));
      }
      if(bestXGOfSector > bestXG)
      {
        bestXG = bestXGOfSector;
        bestSector = lastBestSector;
        bestKickDirection = lastBestKickDirection;
        bestKick = lastBestKick;
        bestKickLength = lastBestKickLength;
      }
      applyRightTeamMateBonus = false;
    }
    //sub bonus to last sector
    else if(sector.type == SectorWheel::Sector::obstacle)
    {
      leftTeammateBonus = -teammateSummand;
      if(applyRightTeamMateBonus)
      {
        bestXGOfSector += -teammateSummand;
        ASSERT(!mapOfRatings.empty());
        mapOfRatings.erase(std::prev(mapOfRatings.end()));
        mapOfRatings.insert(std::pair(i - 1, bestXGOfSector));
      }
      if(bestXGOfSector > bestXG)
      {
        bestXG = bestXGOfSector;
        bestSector = lastBestSector;
        bestKickDirection = lastBestKickDirection;
        bestKick = lastBestKick;
        lastBestKickLength = bestKickLength;
      }
      applyRightTeamMateBonus = false;
    }
    else
    {
      leftTeammateBonus = 0.f;
      applyRightTeamMateBonus = false;
    }
  }
  //save last sector and kick type for hysteresis
  lastKickType = bestKick;
  lastSector = bestSector;
  lastKickLength = bestKickLength;

  if(bestKick != KickInfo::numOfKickTypes)
  {
    theClearTarget.kickAngle = Angle::normalize(bestKickDirection - theRobotPose.rotation);
    theClearTarget.kickTarget = theRobotPose.translation + Vector2f::polar(bestKickLength, bestSector.angleRange.getCenter());
    theClearTarget.rating = getAngleRating(bestSector.angleRange.getCenter(), theFieldBall.positionOnField, bestKickLength);
    theClearTarget.kickType = bestKick;
    theClearTarget.kickLength = bestKickLength;
    theClearTarget.isValid = true;
  }
  else
    theClearTarget.reset();

  //print calculated best value of each free sector
  for(const auto& [index, xG] : mapOfRatings)
  {
    DRAW_TEXT("option:ClearBall:bonus", 1000, 1000 + index * 100, 100, ColorRGBA::violet, index << ": " << xG);
  }
  DRAW_TEXT("option:ClearBall:bonus", 1000, 1000 - 100, 100, ColorRGBA::violet, bestKick);
}
