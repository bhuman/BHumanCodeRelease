/**
 * @file DirectKickOff.cpp
 *
 * This ...
 *
 * @author Arne Hasselbring (from former KickoffStrikerCard.cpp in 2019)
 */

#include "SkillBehaviorControl.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Debugging/DebugDrawings.h"

namespace DirectKickOff
{
  struct ObstacleSector
  {
    Rangea sector; /**< The angular range relative to the ball that the obstacle blocks. */
    float distance; /**< The distance of the obstacle to the ball. */
    float x; /**< The x coordinate on field of the obstacle. */
  };
}
using namespace DirectKickOff;

option((SkillBehaviorControl) DirectKickOff,
       load((float) hysteresisNumber, /**< A number which is used in various places to define a hysteresis offset. */
            (Angle) hysteresisAngle, /**< When sorting sectors by their opening angle, the one selected in the previous frame gets this as bonus. */
            (Angle) goalAngle, /**< No kicks towards the goal allowed. */
            (Angle) minOpeningAngle, /**< The minimum opening angle a sector must have to be considered. */
            (float) ballDistanceForSlowWalk, /**< If the ball is close, walk slow. */
            (float) xPoseForPassReceiver, /**< X coordinate for pass receiver. */
            (int) forceKickAfterTime, /**< Force a kick after this much time passed. */
            (bool) kickOutOfField, /**< Kick out of the field to force a set play. */
            (bool) kickOutOfField2Players, /**< Kick out of field if we have 2 or less players. */
            (float) maxKickLength), /**< Max kick length for normal kick off. */
       vars((bool)(false) wasActive, /**< Whether an in walk kick out of the center circle was already tried. */
            (KickInfo::KickType)(KickInfo::numOfKickTypes) kickType, /**< The kick type to try. */
            (Angle)(0_deg) targetAngle)) /**< The target angle to which the kick-off should go (from the ball in field coordinates). */
{
  initial_state(execute)
  {
    transition
    {
      if(theFrameInfo.getTimeSince(theExtendedGameState.timeWhenStateStarted[GameState::ownKickOff]) > forceKickAfterTime && wasActive)
        goto kick;
    }
    action
    {
      if(theFrameInfo.getTimeSince(theExtendedGameState.timeWhenStateStarted[GameState::ownKickOff]) < forceKickAfterTime)
      {
        LookLeftAndRight({.tilt = theBehaviorParameters.defaultLookDownAngle});
        Stand();
      }
      else
      {
        // Prepare obstacle sectors.
        std::vector<ObstacleSector> obstacleSectors;
        for(const Obstacle& obstacle : theObstacleModel.obstacles)
        {
          const Vector2f obstacleOnField = theRobotPose * obstacle.center;
          if(obstacleOnField.x() < 0.f || obstacleOnField.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius - 300.f))
            continue;

          const float width = (obstacle.left - obstacle.right).norm() + 6.f * theBallSpecification.radius;
          const float distance = std::sqrt(std::max((obstacleOnField - theFieldBall.positionOnField).squaredNorm() - sqr(width / 2.f), 1.f));
          if(distance < theBallSpecification.radius)
            continue;

          const float radius = std::atan(width / (2.f * distance));
          const Angle direction = (obstacleOnField - theFieldBall.positionOnField).angle();

          obstacleSectors.emplace_back();
          obstacleSectors.back().sector = Rangea(Angle::normalize(direction - radius), Angle::normalize(direction + radius));
          obstacleSectors.back().distance = distance;
          obstacleSectors.back().x = obstacleOnField.x();
          // If the previous target is inside this sector, artificially increase its x coordinate so it will potentially be culled before other sectors.
          if(wasActive && obstacleSectors.back().sector.isInside(targetAngle))
            obstacleSectors.back().x += hysteresisNumber;
        }

        obstacleSectors.emplace_back();
        obstacleSectors.back().sector = Rangea(-91_deg, -89_deg);
        obstacleSectors.back().distance = 1000.f;
        obstacleSectors.back().x = 0.f;

        obstacleSectors.emplace_back();
        obstacleSectors.back().sector = Rangea(89_deg, 91_deg);
        obstacleSectors.back().distance = 1000.f;
        obstacleSectors.back().x = 0.f;

        obstacleSectors.emplace_back();
        obstacleSectors.back().sector = Rangea(-goalAngle, goalAngle);
        obstacleSectors.back().distance = 1000.f;
        obstacleSectors.back().x = 0.f;

        // Sort obstacles according to their absolute x coordinate (to ease culling later on).
        std::sort(obstacleSectors.begin(), obstacleSectors.end(), [](const ObstacleSector& s1, const ObstacleSector& s2) { return s1.x < s2.x; });

        SectorWheel wheel;
        std::list<SectorWheel::Sector> sectors;
        bool isLargeEnough = false;
        do
        {
          wheel.begin(theFieldBall.positionOnField);
          for(const ObstacleSector& obstacleSector : obstacleSectors)
            wheel.addSector(obstacleSector.sector, obstacleSector.distance, SectorWheel::Sector::obstacle);
          sectors = wheel.finish();

          for(const SectorWheel::Sector& sector : sectors)
            if(sector.type == SectorWheel::Sector::free &&
               sector.angleRange.getSize() >= ((wasActive && sector.angleRange.isInside(targetAngle)) ? minOpeningAngle : Angle(minOpeningAngle * 2.f)))
            {
              isLargeEnough = true;
              break;
            }
        }
        while(!isLargeEnough && !obstacleSectors.empty() && (obstacleSectors.pop_back(), true));

        DRAW_SECTOR_WHEEL("option:DirectKickOff:wheel", sectors, theFieldBall.endPositionOnField);

        bool passTeammate = false;
        if(theGameState.ownTeam.numOfActivePlayers() > 2)
        {
          for(const auto& t : theGlobalTeammatesModel.teammates)
          {
            Vector2f teammatePose = t.pose.translation;
            if(teammatePose.x() > xPoseForPassReceiver)
            {
              targetAngle = teammatePose.y() > 0 ? 45_deg : -45_deg;
              passTeammate = true;
              break;
            }
          }
        }

        Angle bestOpeningAngle = 0_deg;
        Angle bestTargetAngle = 0_deg;
        Angle bestCenter = 180_deg;
        bool foundTeammateAngle = false;

        for(const SectorWheel::Sector& sector : sectors)
        {
          if(std::abs(sector.angleRange.getCenter()) > 90_deg
             || sector.type != SectorWheel::Sector::free)
            continue;
          const Angle openingAngle = sector.angleRange.getSize() + ((wasActive && sector.angleRange.isInside(targetAngle)) ? hysteresisAngle : 0_deg);
          const bool betterCenter = std::abs(sector.angleRange.getCenter()) < std::abs(bestCenter);
          const bool teammateSide = passTeammate && sector.angleRange.getCenter() * targetAngle >= 0.f;

          if((betterCenter && (!foundTeammateAngle || teammateSide))
             || (teammateSide && !foundTeammateAngle))
          {
            foundTeammateAngle |= sector.angleRange.getCenter() * targetAngle >= 0.f && passTeammate;
            bestCenter = sector.angleRange.getCenter();
            bestOpeningAngle = openingAngle;
            // kickOutOfField: as close to forward as possible, otherwise soft and far away from opponent
            bestTargetAngle = kickOutOfField || (kickOutOfField2Players && theGameState.ownTeam.numOfActivePlayers() <= 2) ? sector.angleRange.limit(0) : sector.angleRange.getCenter();
          }
        }

        ASSERT(bestOpeningAngle > 0_deg);

        targetAngle = bestTargetAngle;
        float kickLength = Vector2f(theFieldDimensions.xPosOpponentGoal / 2.f, theFieldDimensions.yPosLeftTouchline / 2.f).norm();
        kickType = (!kickOutOfField || (kickOutOfField2Players && theGameState.ownTeam.numOfActivePlayers() <= 2)) ? KickInfo::rlKickDynamicOld : KickInfo::rlKickDynamic;

        ASSERT(kickType != KickInfo::numOfKickTypes);
        GoToBallAndKick({.targetDirection = Angle::normalize(targetAngle - theRobotPose.rotation),
                         .kickType = kickType,
                         .lookActiveWithBall = true,
                         .length = kickLength,
                         .reduceWalkSpeedType = theFieldBall.positionRelative.squaredNorm() < sqr(ballDistanceForSlowWalk) ? ReduceWalkSpeedType::slow : ReduceWalkSpeedType::noChange});
        wasActive = true;
      }
    }
  }

  state(kick)
  {
    action
    {
      const float kickLength = kickOutOfField || (kickOutOfField2Players && theGameState.ownTeam.numOfActivePlayers() <= 2)
                               ? std::numeric_limits<float>::max()
                               : std::min(maxKickLength, Vector2f(theFieldDimensions.xPosOpponentGoal / 2.f, theFieldDimensions.yPosLeftTouchline / 2.f).norm());
      kickType = (!kickOutOfField || (kickOutOfField2Players && theGameState.ownTeam.numOfActivePlayers() <= 2)) ? KickInfo::rlKickDynamicOld : KickInfo::rlKickDynamic;

      GoToBallAndKick({.targetDirection = Angle::normalize(targetAngle - theRobotPose.rotation),
                       .kickType = kickType,
                       .lookActiveWithBall = true,
                       .length = kickLength,
                       .reduceWalkSpeedType = theFieldBall.positionRelative.squaredNorm() < sqr(ballDistanceForSlowWalk) ? ReduceWalkSpeedType::slow : ReduceWalkSpeedType::noChange });
    }
  }
}
