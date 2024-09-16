/**
 * @file KickAtGoal.cpp
 *
 * This file defines an implementation of the KickAtGoal skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Debugging/DebugDrawings.h"
#include <limits>

namespace KickAtGoal
{
  struct ObstacleSector
  {
    Rangea sector; /**< The angular range relative to the ball that the obstacle blocks. */
    float distance; /**< The distance of the obstacle to the ball. */
    float x; /**< The x coordinate on field of the obstacle. */
  };
}
using namespace KickAtGoal;

option((SkillBehaviorControl) KickAtGoal,
       load((float) hysteresisNumber, /**< A number which is used in various places to define a hysteresis offset. */
            (float) hysteresisKickRangeExtension, /**< The range of the previously selected kick is extended by this length. */
            (Angle) hysteresisAngle, /**< When sorting sectors by their opening angle, the one selected in the previous frame gets this as bonus. */
            (Angle) minOpeningAngle, /**< The minimum opening angle a sector must have to be considered. */
            (Angle) kickInaccuracy, /**< The search angle range is narrowed by this to both sides. */
            (float) hysteresisDecisionPenalty, /**< Changing the decision of the previous frame results in this penalty to the time. */
            (float) walkForwardLongMalus, /**< Malus for the range. */
            (std::vector<KickInfo::KickType>) allowedKicks,
            (std::vector<Rangef>) allowedKickRanges), /**< Kicking at the goal is only allowed from these distances. */
       vars((bool)(false) aimingAtGoal, /**< Whether the robot is able to score a goal in the current state. */
            (Vector2f)(Vector2f::Zero()) goalLineIntersection, /**< The point where the ball would have intersected the goal line according to the current plan. */
            (KickInfo::KickType)(KickInfo::numOfKickTypes) kickType, /**< The selected kick type to kick into the goal. */
            (Angle)(0_deg) kickDirectionRelative, /**< The pose where the robot has to walk to kick into the goal. */
            (Rangea)(Rangea(0_deg, 0_deg)) precisionRange)) /**< The precision of the kick direction. */
{
  const Vector2f leftGoalPost(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal); // The position of the left post of the opponent's goal.
  const Vector2f rightGoalPost(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal); // The position of the right post of the opponent's goal.
  const Geometry::Line goalLine(Vector2f(theFieldDimensions.xPosOpponentGoalLine + theFieldDimensions.fieldLinesWidth * 0.5f + theBallSpecification.radius, 0.f), Vector2f(0.f, 1.f)); // The line which the ball has to cross to be inside the goal.

  bool kickHasBeenCalculated = false; // Whether the kick type, direction etc. have already been calculated in this frame.

  const auto calcAvailableKicks = [&](KickInfo::KickType lastKickType) -> std::vector<KickInfo::KickType>
  {
    std::vector<KickInfo::KickType> kicks;
    kicks.reserve(KickInfo::numOfKickTypes);

    const float distanceToGoal = (Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f) - theFieldBall.positionOnField).norm();

    // Check for each kick, if the specific robot is allowed to execute this one
    for(const auto kick : allowedKicks)
    {
      switch(kick)
      {
        case KickInfo::forwardFastLeft:
        case KickInfo::forwardFastLeftLong:
        {
          if(theDamageConfigurationBody.sides[Legs::right].weakLeg)
            continue; // skip this kick
          break;
        }
        case KickInfo::forwardFastRight:
        case KickInfo::forwardFastRightLong:
        {
          if(theDamageConfigurationBody.sides[Legs::left].weakLeg)
            continue; // skip this kick
          break;
        }
        default:
          break;
      }
      // Check for each kick, if its kick range is inside the allowed kick ranges AND if the distance to the goal (from the ball) is also inside this allowed kick range
      bool isInside = false;
      for(const auto& range : allowedKickRanges)
      {
        ASSERT(range.min < range.max);
        isInside |= range.isInside(theKickInfo[kick].range.max) && range.isInside(distanceToGoal);
        if(!isInside && kick == lastKickType) // Make sure when we decided once for the kick, we do not oscillate the decision!
        {
          Rangef otherRange = range;
          otherRange.max += hysteresisKickRangeExtension;
          otherRange.min -= hysteresisKickRangeExtension;
          isInside |= otherRange.isInside(theKickInfo[kick].range.max) && otherRange.isInside(distanceToGoal);
        }
      }
      if(isInside)
        kicks.push_back(kick);
    }

    return kicks;
  };

  const auto updateKickDirectionAndKick = [&]
  {
    if(kickHasBeenCalculated)
      return;

    const Angle lastTargetAngleOnField = (goalLineIntersection - theFieldInterceptBall.interceptedEndPositionOnField).angle();
    const KickInfo::KickType lastKickType = aimingAtGoal ? kickType : KickInfo::numOfKickTypes;
    const bool lastAimingAtGoal = aimingAtGoal;
    kickType = KickInfo::numOfKickTypes;
    aimingAtGoal = false;
    Rangea bestPrecisionRange(0_deg, 0_deg);

    const float minBallGoalPostOffset = theFieldDimensions.goalPostRadius + theBallSpecification.radius;
    const Angle leftAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (leftGoalPost - theFieldInterceptBall.interceptedEndPositionOnField).norm()));
    const Angle rightAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffset / (rightGoalPost - theFieldInterceptBall.interceptedEndPositionOnField).norm()));
    const Angle angleToLeftPost = (leftGoalPost - theFieldInterceptBall.interceptedEndPositionOnField).angle() - leftAngleOffset;
    const Angle angleToRightPost = (rightGoalPost - theFieldInterceptBall.interceptedEndPositionOnField).angle() + rightAngleOffset;

    RAY("option:KickAtGoal:goalSector", theFieldInterceptBall.interceptedEndPositionOnField, angleToLeftPost, 10, Drawings::solidPen, ColorRGBA::blue);
    RAY("option:KickAtGoal:goalSector", theFieldInterceptBall.interceptedEndPositionOnField, angleToRightPost, 10, Drawings::solidPen, ColorRGBA::red);

    auto kicks = calcAvailableKicks(lastKickType);
    if(kicks.empty())
    {
      aimingAtGoal = false;
      return;
    }

    if(theFieldInterceptBall.interceptedEndPositionOnField.x() > theFieldDimensions.xPosOpponentGoalPost - minBallGoalPostOffset - (lastAimingAtGoal ? hysteresisNumber : 0.f) &&
       std::abs(theFieldInterceptBall.interceptedEndPositionOnField.y()) < theFieldDimensions.yPosLeftGoal - minBallGoalPostOffset - (lastAimingAtGoal ? 0.f : hysteresisNumber))
    {
      KickInfo::KickType bestKickType = KickInfo::numOfKickTypes;
      Pose2f bestKickPoseRelative;
      float timeToReachBestKickPose = std::numeric_limits<float>::max();

      const Rangea clampedRange(std::max<Angle>(angleToRightPost, -pi_2), std::min<Angle>(angleToLeftPost, pi_2));
      const Rangea searchRange = clampedRange.getSize() > 2.f * kickInaccuracy ?
                                 Rangea(clampedRange.min + kickInaccuracy, clampedRange.max - kickInaccuracy) :
                                 Rangea(clampedRange.getCenter());

      for(KickInfo::KickType kick : kicks)
      {
        const Pose2f kickPose = theRobotPose.inverse() * KickSelection::calcOptimalKickPoseForTargetAngleRange(searchRange, theRobotPose, theFieldInterceptBall.interceptedEndPositionOnField, theKickInfo[kick].ballOffset, theKickInfo[kick].rotationOffset);
        const float ttrp = KickSelection::calcTTRP(kickPose, theWalkingEngineOutput.maxSpeed) + theKickInfo[kick].executionTime + (kick == lastKickType ? 0.f : hysteresisDecisionPenalty);

        if(ttrp < timeToReachBestKickPose)
        {
          timeToReachBestKickPose = ttrp;
          bestKickPoseRelative = kickPose;
          bestKickType = kick;
          const Angle kickAngleInField = Angle::normalize(theRobotPose.rotation + bestKickPoseRelative.rotation - theKickInfo[bestKickType].rotationOffset);
          bestPrecisionRange = Rangea(std::min(searchRange.min - kickAngleInField, 0.f), std::max(searchRange.max - kickAngleInField, 0.f));
        }
      }

      kickDirectionRelative = Angle::normalize(bestKickPoseRelative.rotation - theKickInfo[bestKickType].rotationOffset);
      kickType = bestKickType;
      precisionRange = bestPrecisionRange;
      VERIFY(Geometry::getIntersectionOfLines(goalLine,
                                              Geometry::Line(Pose2f(Angle::normalize(theRobotPose.rotation + kickDirectionRelative),
                                                             theFieldInterceptBall.interceptedEndPositionOnField)), goalLineIntersection));
      CROSS("option:KickAtGoal:goalLineIntersection", goalLineIntersection.x(), goalLineIntersection.y(), 100, 20, Drawings::solidPen, ColorRGBA::white);
      aimingAtGoal = true;
    }
    else if(angleToLeftPost < angleToRightPost ||
            (theFieldInterceptBall.interceptedEndPositionOnField.x() >= theFieldDimensions.xPosOpponentGoalPost - minBallGoalPostOffset - (lastAimingAtGoal ? 0.f : hysteresisNumber) &&
             std::abs(theFieldInterceptBall.interceptedEndPositionOnField.y()) >= theFieldDimensions.yPosLeftGoal - minBallGoalPostOffset - (lastAimingAtGoal ? 0.f : hysteresisNumber)))
    {
      aimingAtGoal = false;
    }
    else
    {
      ASSERT(std::abs(angleToLeftPost) <= pi_2);
      ASSERT(std::abs(angleToRightPost) <= pi_2);

      // Prepare obstacle sectors.
      std::vector<ObstacleSector> obstacleSectors;
      for(const Obstacle& obstacle : theObstacleModel.obstacles)
      {
        const Vector2f obstacleOnField = theRobotPose * obstacle.center;
        if(obstacleOnField.x() > (theFieldDimensions.xPosOpponentGoalLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
          continue;

        const float width = (obstacle.left - obstacle.right).norm() + 4.f * theBallSpecification.radius;
        const float distance = std::sqrt(std::max((obstacleOnField - theFieldInterceptBall.interceptedEndPositionOnField).squaredNorm() - sqr(width / 2.f), 1.f));
        if(distance < theBallSpecification.radius)
          continue;

        const float radius = std::atan(width / (2.f * distance));
        const Angle direction = (obstacleOnField - theFieldInterceptBall.interceptedEndPositionOnField).angle();
        // Cull obstacles that are not between ball and goal anyway.
        // This works unnormalized because |angleToLeftPost| and |angleToRightPost| are <= pi_2 and radius <= pi_2
        if(direction - radius > angleToLeftPost || direction + radius < angleToRightPost)
          continue;
        obstacleSectors.emplace_back();
        obstacleSectors.back().sector = Rangea(Angle::normalize(direction - radius), Angle::normalize(direction + radius));
        obstacleSectors.back().distance = distance;
        obstacleSectors.back().x = obstacleOnField.x();
        // If the previous target is inside this sector, artificially increase its x coordinate so it will potentially be culled before other sectors.
        if(lastAimingAtGoal && obstacleSectors.back().sector.isInside(lastTargetAngleOnField))
          obstacleSectors.back().x += hysteresisNumber;
      }

      // Sort obstacles according to their absolute x coordinate (to ease culling later on).
      std::sort(obstacleSectors.begin(), obstacleSectors.end(), [](const ObstacleSector& s1, const ObstacleSector& s2) { return s1.x < s2.x; });

      // Determine the x coordinate up to which obstacles may be culled.
      const float cullFactor = std::max(0.f, std::min(theFieldInterceptBall.interceptedEndPositionOnField.x() / theFieldDimensions.xPosOpponentPenaltyMark, 1.f));
      const float cullBeyondX = cullFactor * theFieldDimensions.xPosOpponentGoalLine +
                                (1.f - cullFactor) * theFieldDimensions.xPosOpponentPenaltyMark;
      LINE("option:KickAtGoal:cullLine", cullBeyondX, theFieldDimensions.yPosLeftTouchline, cullBeyondX, theFieldDimensions.yPosRightTouchline, 10, Drawings::solidPen, ColorRGBA::black);

      SectorWheel wheel;
      std::list<SectorWheel::Sector> sectors;
      bool isLargeEnough = false;
      do
      {
        wheel.begin(theFieldInterceptBall.interceptedEndPositionOnField);
        wheel.addSector(Rangea(angleToRightPost, angleToLeftPost), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);
        for(const ObstacleSector& obstacleSector : obstacleSectors)
          wheel.addSector(obstacleSector.sector, obstacleSector.distance, SectorWheel::Sector::obstacle);
        sectors = wheel.finish();

        for(const SectorWheel::Sector& sector : sectors)
          if(sector.type == SectorWheel::Sector::goal &&
             sector.angleRange.getSize() >= ((lastAimingAtGoal && sector.angleRange.isInside(lastTargetAngleOnField)) ? Angle(minOpeningAngle * 0.5f) : minOpeningAngle))
          {
            isLargeEnough = true;
            break;
          }
      }
      while(!isLargeEnough && !obstacleSectors.empty() && obstacleSectors.back().x > cullBeyondX && (obstacleSectors.pop_back(), true));

      DRAW_SECTOR_WHEEL("option:KickAtGoal:wheel", sectors, theFieldInterceptBall.interceptedEndPositionOnField);

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
        const Angle wholeOpeningAngle = sector.angleRange.getSize() + ((lastAimingAtGoal && sector.angleRange.isInside(lastTargetAngleOnField)) ? minOpeningAngle * 0.5f : 0.f);
        if(wholeOpeningAngle < minOpeningAngle)
          continue;

        KickInfo::KickType bestKickType = KickInfo::numOfKickTypes;
        Pose2f bestKickPoseRelative;
        float timeToReachBestKickPose = std::numeric_limits<float>::max();

        for(KickInfo::KickType kick : kicks)
        {
          float usedRange = theKickInfo[kick].range.max + ((kick == lastKickType) ? hysteresisKickRangeExtension : 0.f);
          usedRange += theKickInfo[kick].walkKickType == WalkKicks::forwardLong ? walkForwardLongMalus : 0.f;
          if(usedRange < (goalLine.base.x() - theFieldInterceptBall.interceptedEndPositionOnField.x()))
            continue;

          const Angle absAngle = std::acos((goalLine.base.x() - theFieldInterceptBall.interceptedEndPositionOnField.x()) / usedRange);
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
          const Pose2f kickPose = theRobotPose.inverse() * KickSelection::calcOptimalKickPoseForTargetAngleRange(searchRange, theRobotPose, theFieldInterceptBall.interceptedEndPositionOnField, theKickInfo[kick].ballOffset, theKickInfo[kick].rotationOffset);
          const float ttrp = KickSelection::calcTTRP(kickPose, theWalkingEngineOutput.maxSpeed) + theKickInfo[kick].executionTime + (kick == lastKickType ? 0.f : hysteresisDecisionPenalty);

          if(ttrp < timeToReachBestKickPose)
          {
            timeToReachBestKickPose = ttrp;
            bestKickPoseRelative = kickPose;
            bestKickType = kick;
            const Angle kickAngleInField = Angle::normalize(theRobotPose.rotation + bestKickPoseRelative.rotation - theKickInfo[bestKickType].rotationOffset);
            bestPrecisionRange = Rangea(std::min(searchRange.min - kickAngleInField, 0.f), std::max(searchRange.max - kickAngleInField, 0.f));
          }
        }

        if(bestKickType != KickInfo::numOfKickTypes)
        {
          kickDirectionRelative = Angle::normalize(bestKickPoseRelative.rotation - theKickInfo[bestKickType].rotationOffset);
          kickType = bestKickType;
          precisionRange = bestPrecisionRange;
          VERIFY(Geometry::getIntersectionOfLines(goalLine,
                                                  Geometry::Line(Pose2f(Angle::normalize(theRobotPose.rotation + kickDirectionRelative),
                                                                 theFieldInterceptBall.interceptedEndPositionOnField)), goalLineIntersection));
          CROSS("option:KickAtGoal:goalLineIntersection", goalLineIntersection.x(), goalLineIntersection.y(), 100, 20, Drawings::solidPen, ColorRGBA::white);
          aimingAtGoal = true;
          break;
        }
      }
    }

    kickHasBeenCalculated = true;
  };

  initial_state(execute)
  {
    action
    {
      updateKickDirectionAndKick();

      const bool allowDirectKick = !(theGameState.isFreeKick() &&
                                     theGameState.isForOwnTeam());
      if(aimingAtGoal && allowDirectKick)
      {
        GoToBallAndKick({.targetDirection = kickDirectionRelative,
                         .kickType = kickType,
                         .alignPrecisely = theKickInfo[kickType].motion == MotionPhase::walk ? KickPrecision::notPrecise : KickPrecision::precise,
                         .directionPrecision = precisionRange });
      }
      else
        DribbleToGoal();

      if(!kickHasBeenCalculated)
        aimingAtGoal = false;
    }
  }
};
