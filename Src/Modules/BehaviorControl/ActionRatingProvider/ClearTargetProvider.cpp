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
  theClearTarget.reset = [this]() -> void
  {
    lastKickType = KickInfo::numOfKickTypes;
    lastSector = {};
    timeWhenBestKickWasUpdated = 0;
  };

  theClearTarget.getAngle = [this]() -> Angle
  {
    return Angle::normalize(bestKickPoseRelative.rotation - theKickInfo[bestKick].rotationOffset);
  };

  theClearTarget.getKickType = [this]() -> KickInfo::KickType
  {
    if(theFrameInfo.time != timeWhenBestKickWasUpdated)
    {
      calcBestKick();
      timeWhenBestKickWasUpdated = theFrameInfo.time;
    }
    return bestKick;
  };

  theClearTarget.getTarget = [this]() -> Vector2f
  {
    return theRobotPose.translation + Vector2f::polar(theKickInfo[bestKick].range.max, bestSector.angleRange.getCenter());
  };
  theClearTarget.getRating = [this]() -> float
  {
    if(bestKick != KickInfo::numOfKickTypes)
    {
      return getAngleRating(bestSector.angleRange.getCenter(), theFieldBall.positionOnField, theKickInfo[bestKick].range.max);
    }
    return -1.0f;
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
  if(targetPosition.x() > theFieldDimensions.xPosOwnGoalLine + minDistanceToXLine
     && targetPosition.x() < theFieldDimensions.xPosOpponentGoalLine - minDistanceToXLine
     && targetPosition.y() < theFieldDimensions.yPosLeftFieldBorder - minDistanceToYLine
     && targetPosition.y() > theFieldDimensions.yPosRightFieldBorder + minDistanceToYLine)
  {
    return true;
  }
  return false;
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
  const float goalRating = theExpectedGoals.getRating(targetPosition, false);

  //calculate field factor of current position
  //the factor is 1 in ownPenaltyArea, 1...0 between ownPenaltyArea and HalfwayLine and 0 in oppositeHalf
  //except when it is a own freekick
  float fieldFactor = mapToRange(theFieldBall.positionOnField.x(), theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosHalfwayLine, 1.f, 0.f);
  if(theGameState.isFreeKick() && theGameState.isForOwnTeam())
  {
    fieldFactor = 1.f;
  }

  COMPLEX_DRAWING("option:ClearBall:evaluation")
  {
    std::string closer = "";
    if(isTeammateCloseToBall(candidateAngle, basePosition, targetDistance))
    {
      closer = "*";
    }
    const Vector2f& ballPositionOnField = theFieldInterceptBall.interceptedEndPositionOnField;
    LINE("option:ClearBall:evaluation", ballPositionOnField.x(), ballPositionOnField.y(), targetPosition.x(), targetPosition.y(), 20, Drawings::PenStyle::solidPen, ColorRGBA::violet);
    DRAW_TEXT("option:ClearBall:evaluation", targetPosition.x(), targetPosition.y() - 250, 200, ColorRGBA::violet, number << ": " << goalRating * fieldFactor << closer);
  }
  //return xG-value of the target position times field factor of the current position
  return goalRating * fieldFactor;
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

void ClearTargetProvider::calcBestKick()
{
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

  for(const Obstacle& obstacle : theObstacleModel.obstacles)
  {
    const Vector2f obstacleOnField = theRobotPose * obstacle.center;

    // There may be a lot of "low-quality" obstacles behind on the border strip.
    if(obstacleOnField.x() < theFieldDimensions.xPosOwnGoalLine + 300.f)
      continue;

    const float width = (obstacle.left - obstacle.right).norm() + 4.f * theBallSpecification.radius;
    const float distance = std::sqrt(std::max((obstacleOnField - theFieldInterceptBall.interceptedEndPositionOnField).squaredNorm() - sqr(width / 2.f), 1.f));
    if(distance < theBallSpecification.radius)
      continue;

    const float radius = std::atan(width / (2.f * distance));
    const Angle direction = (obstacleOnField - theFieldInterceptBall.interceptedEndPositionOnField).angle();
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
  //add extra kicks since it is a set piece
  if(theGameState.isFreeKick() && theGameState.isForOwnTeam())
  {
    kickTypes.reserve(kickTypes.size() + extraKicksSetPieces.size());
    kickTypes.insert(kickTypes.end(), extraKicksSetPieces.begin(), extraKicksSetPieces.end());
  }
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
        createSmallSectors(var, (Angle)pi, newSmallSectors, newSmallSector);
        if(pi - var > smallSector)
        {
          newSmallSector.angleRange.min = var;
          newSmallSector.angleRange.max = pi;
          newSmallSectors.push_back(newSmallSector);
        }
        //we add values to get away from -pi
        var = -pi;
        createSmallSectors(var, end, newSmallSectors, newSmallSector);
        if(end - var > smallSector)
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

  float bestXG = 0.f;//best xG of all sectors and kicktypes
  float leftTeammateBonus = 0.0f; //when the current sector is obstacle or teammate we add or sub a bonus to the next sector if it is a free one
  bool applyRightTeamMateBonus = false;//when the current sector is free, we let the next sector know, that it has to add or sub a bonus to the current sector if next sector is obstacle/teammate
  float goalRating = 0.f;//xG of current sector and shottype
  KickInfo::KickType lastBestKick = KickInfo::numOfKickTypes; //kicktype of last sector
  SectorWheel::Sector lastBestSector; //last sector
  Pose2f lastBestKickPoseRelative;//pose of last sector
  float bestXGOfSector = 0.0f;//best XG of last sector
  std::map<int, float> mapOfRatings;//needed for debug drawings
  COMPLEX_DRAWING("option:ClearBall:Zero")
  {
    const Vector2f targetPosition = theFieldBall.positionOnField + Vector2f::polar(10000, -pi);
    const Vector2f& ballPositionOnField = theFieldInterceptBall.interceptedEndPositionOnField;
    LINE("option:ClearBall:Zero", ballPositionOnField.x(), ballPositionOnField.y(), targetPosition.x(), targetPosition.y(), 40, Drawings::PenStyle::dottedPen, ColorRGBA::yellow);
  }
  int i = 0;
  //skip big sectors
  //add the small sectors at the end of the structure
  for(const SectorWheel::Sector& sector : sectors)
  {
    i++;
    if(sector.type == SectorWheel::Sector::free)
    {
      bool close = false;
      bestXGOfSector = 0.0f;
      //sectors with range < 10_deg are too small to pass into
      if(sector.angleRange.getSize() > smallSector)
      {
        //check best kick for the current sector
        for(KickInfo::KickType kickType : kickTypes)
        {
          const Angle center = sector.angleRange.getCenter();
          goalRating = getAngleRating(center, theFieldBall.positionOnField, theKickInfo[kickType].range.max, i);
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
          const Pose2f kickPoseRelativeSize = theRobotPose.inverse() * KickSelection::calcOptimalKickPoseForTargetAngleRange(sector.angleRange, theRobotPose, theFieldBall.positionOnField, theKickInfo[kickType].ballOffset, theKickInfo[kickType].rotationOffset);
          //save best rating of this sector
          if(goalRating > bestXGOfSector)
          {
            bestXGOfSector = goalRating;
          }
          //check if clearTarget is useful
          close = isTeammateCloseToBall(sector.angleRange.getCenter(), theFieldBall.positionOnField, theKickInfo[kickType].range.max);
          if(goalRating > bestXG && close)
          {
            bestXG = goalRating;
            bestSector = sector;
            bestKickPoseRelative = kickPoseRelativeSize;
            bestKick = kickType;
            lastBestSector = sector;
            lastBestKickPoseRelative = kickPoseRelativeSize;
            lastBestKick = kickType;
          }
        }
      }
      //reset teammate bonus since this one is a free sector
      leftTeammateBonus = 0.f;
      applyRightTeamMateBonus = true;

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
        bestKickPoseRelative = lastBestKickPoseRelative;
        bestKick = lastBestKick;
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
        bestKickPoseRelative = lastBestKickPoseRelative;
        bestKick = lastBestKick;
      }
      applyRightTeamMateBonus = false;
    }
    else
    {
      leftTeammateBonus = 0.f;
      applyRightTeamMateBonus = false;
    }
  }
  //save last sector and kicktype for hysteresis
  lastKickType = bestKick;
  lastSector = bestSector;
  //print calculated best value of each free sector
  for(const auto& [index, xG] : mapOfRatings)
  {
    DRAW_TEXT("option:ClearBall:bonus", 1000, 1000 + index * 100, 100, ColorRGBA::violet, index << ": " << xG);
  }
  DRAW_TEXT("option:ClearBall:bonus", 1000, 1000 - 100, 100, ColorRGBA::violet, bestKick);
}
