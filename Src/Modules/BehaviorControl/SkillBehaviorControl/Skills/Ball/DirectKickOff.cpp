/**
 * @file DirectKickOff.cpp
 *
 * This ...
 *
 * @author Arne Hasselbring (from former KickoffStrikerCard.cpp in 2019).
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Debugging/DebugDrawings.h"

SKILL_IMPLEMENTATION(DirectKickOffImpl,
{,
  IMPLEMENTS(DirectKickOff),
  CALLS(GoToBallAndKick),
  CALLS(LookLeftAndRight),
  CALLS(Stand),
  REQUIRES(BallSpecification),
  REQUIRES(ExtendedGameState),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(KickInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (float)(300.f) hysteresisNumber, /**< A number which is used in various places to define a hysteresis offset. */
    (Angle)(45_deg) hysteresisAngle, /**< When sorting sectors by their opening angle, the one selected in the previous frame gets this as bonus. */
    (Angle)(60_deg) halfGoalSectorAngle, /**< Half of the goal opening angle around the positive x axis. */
    (Angle)(20_deg) minOpeningAngle, /**< The minimum opening angle a sector must have to be considered. */
  }),

});

class DirectKickOffImpl : public DirectKickOffImplBase
{
  void execute(const DirectKickOff&) override
  {
    if(theFrameInfo.getTimeSince(theExtendedGameState.timeWhenStateStarted[GameState::ownKickOff]) < 2000)
    {
      theLookLeftAndRightSkill();
      theStandSkill();
      return;
    }

    // Prepare obstacle sectors.
    std::vector<ObstacleSector> obstacleSectors;
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
    {
      const Vector2f obstacleOnField = theRobotPose * obstacle.center;
      if(obstacleOnField.x() < 0.f || obstacleOnField.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius - 300.f))
        continue;

      const float width = (obstacle.left - obstacle.right).norm() + 4.f * theBallSpecification.radius;
      const float distance = std::sqrt(std::max((obstacleOnField - theFieldBall.positionOnField).squaredNorm() - sqr(width / 2.f), 1.f));
      if(distance < theBallSpecification.radius)
        continue;

      const float radius = std::atan(width / (2.f * distance));
      const Angle direction = (obstacleOnField - theFieldBall.positionOnField).angle();
      // Cull obstacles that are not in the goal sector anyway.
      if(direction - radius > halfGoalSectorAngle || direction + radius < -halfGoalSectorAngle)
        continue;
      obstacleSectors.emplace_back();
      obstacleSectors.back().sector = Rangea(Angle::normalize(direction - radius), Angle::normalize(direction + radius));
      obstacleSectors.back().distance = distance;
      obstacleSectors.back().x = obstacleOnField.x();
      // If the previous target is inside this sector, artificially increase its x coordinate so it will potentially be culled before other sectors.
      if(wasActive && obstacleSectors.back().sector.isInside(targetAngle))
        obstacleSectors.back().x += hysteresisNumber;
    }

    // Sort obstacles according to their absolute x coordinate (to ease culling later on).
    std::sort(obstacleSectors.begin(), obstacleSectors.end(), [](const ObstacleSector& s1, const ObstacleSector& s2) { return s1.x < s2.x; });

    SectorWheel wheel;
    std::list<SectorWheel::Sector> sectors;
    bool isLargeEnough = false;
    do
    {
      wheel.begin(theFieldBall.positionOnField);
      wheel.addSector(Rangea(-halfGoalSectorAngle, halfGoalSectorAngle), 2.f * theKickInfo[KickInfo::walkForwardsLeftAlternative].range.max, SectorWheel::Sector::goal);
      for(const ObstacleSector& obstacleSector : obstacleSectors)
        wheel.addSector(obstacleSector.sector, obstacleSector.distance, SectorWheel::Sector::obstacle);
      sectors = wheel.finish();

      for(const SectorWheel::Sector& sector : sectors)
        if(sector.type == SectorWheel::Sector::goal &&
           sector.angleRange.getSize() >= ((wasActive && sector.angleRange.isInside(targetAngle)) ? Angle(minOpeningAngle * 0.5f) : minOpeningAngle))
        {
          isLargeEnough = true;
          break;
        }
    }
    while(!isLargeEnough && !obstacleSectors.empty() && (obstacleSectors.pop_back(), true));

    DRAW_SECTOR_WHEEL("skill:DirectKickOff:wheel", sectors, theFieldBall.endPositionOnField);

    Angle bestOpeningAngle = 0_deg;
    Angle bestTargetAngle = 0_deg;
    for(const SectorWheel::Sector& sector : sectors)
    {
      if(sector.type != SectorWheel::Sector::goal)
        continue;
      const Angle openingAngle = sector.angleRange.getSize() + ((wasActive && sector.angleRange.isInside(targetAngle)) ? hysteresisAngle : 0_deg);
      if(openingAngle > bestOpeningAngle)
      {
        bestOpeningAngle = openingAngle;
        bestTargetAngle = sector.angleRange.getCenter();
      }
    }

    ASSERT(bestOpeningAngle > 0_deg);

    targetAngle = bestTargetAngle;
    if(targetAngle > (wasActive ? (kickType == KickInfo::walkForwardsRightAlternative ? -hysteresisAngle : hysteresisAngle) : 0_deg))
      kickType = KickInfo::walkForwardsRightAlternative;
    else
      kickType = KickInfo::walkForwardsLeftAlternative;

    theGoToBallAndKickSkill({.targetDirection = Angle::normalize(targetAngle - theRobotPose.rotation),
                             .kickType = kickType,
                             .lookActiveWithBall = true});
    wasActive = true;
  }

  void reset(const DirectKickOff&) override
  {
    wasActive = false;
    kickType = KickInfo::walkForwardsRightAlternative;
  }

  void preProcess(const DirectKickOff&) override
  {
    DECLARE_DEBUG_DRAWING("skill:DirectKickOff:wheel", "drawingOnField");
  }

  void preProcess() override {}

  struct ObstacleSector
  {
    Rangea sector; /**< The angular range relative to the ball that the obstacle blocks. */
    float distance; /**< The distance of the obstacle to the ball. */
    float x; /**< The x coordinate on field of the obstacle. */
  };

  bool wasActive; /**< Whether an in walk kick out of the center circle was already tried. */
  KickInfo::KickType kickType; /**< The kick type to try. */
  Angle targetAngle; /**< The target angle to which the kick-off should go (from the ball in field coordinates). */
};

MAKE_SKILL_IMPLEMENTATION(DirectKickOffImpl);
