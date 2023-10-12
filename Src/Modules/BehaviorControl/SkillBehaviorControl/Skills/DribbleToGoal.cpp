/**
 * @file DribbleToGoalFieldRatingCard.cpp
 *
 * This file implements a card that dribbles the ball to the goal.
 * It is always runnable for a striker.
 *
 * @author Philip Reichenberg
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldRating.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Debugging/DebugDrawings.h"
#include <map>

SKILL_IMPLEMENTATION(DribbleToGoalImpl,
{,
  IMPLEMENTS(DribbleToGoal),
  CALLS(GoToBallAndDribble),
  REQUIRES(BallSpecification),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldRating),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (int)(100) iterationSteps, /**< Num of iteration steps. */
    (float)(10.f) stepLength, /**< Iterate this much per step. */
    (float)(1200.f) obstacleDistance, /**< Ignore obstacles further away than this distance. */
    (Angle)(3_deg) minFreeSector,
    (float)(1000.f) dribbleRange, /**< Planned look ahead dribble kick range. */
    (float)(500.f) safeDistanceToFieldBoarder, /**< Dribble range + this threshold shall not touch the field border. */
    (float)(500.f) lookActiveMinBallDistance, /**< If the ball is at least this far away, use lookActive with withBall = true. */
    (float)(15.f) searchStepDrawWidth,
    (float)(5.f) searchStepDrawScale,
    (int)(10) searchStepDrawModulo,
  }),
});

class DribbleToGoalImpl : public DribbleToGoalImplBase
{
  void execute(const DribbleToGoal&) override
  {
    if(theFieldBall.positionRelative.squaredNorm() > sqr(500.f))
    {
      theGoToBallAndDribbleSkill({.targetDirection = -theRobotPose.rotation});
      return;
    }

    PotentialValue pv;
    lastDribbleAngle = Angle::normalize(lastDribbleAngle + theRobotPose.rotation);
    // clip ball inside the field
    Vector2f position = theFieldBall.interceptedEndPositionOnField;
    xMinMax.clamp(position.x());
    yMinMax.clamp(position.y());
    const Vector2f startPosition = position - position.normalized(0.001f); // To ensure we are not on the field line
    position += Vector2f::polar(100.f, lastDribbleAngle);
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

      COMPLEX_DRAWING("skill:DribbleToGoal:step")
      {
        if(i % searchStepDrawModulo == 0 && i != 0) // skip first
          ARROW("skill:DribbleToGoal:step", position.x(), position.y(), position.x() + searchStepDrawScale * direction.x(), position.y() + searchStepDrawScale * direction.y(), searchStepDrawWidth, Drawings::solidPen, ColorRGBA::black);
      }

      position += direction;
      if(position.x() > theFieldDimensions.xPosOpponentGroundLine && position.y() < theFieldDimensions.yPosLeftGoal && position.y() > theFieldDimensions.yPosRightGoal)
        break;
    }

    // get dribble angle
    Angle dribbleAngleInField = (position - startPosition).angle();
    calculateSectorWheel();

    COMPLEX_DRAWING("skill:DribbleToGoal:direction")
    {
      const Vector2f pos2 = theFieldBall.interceptedEndPositionOnField + Vector2f::polar(300.f, dribbleAngleInField);
      ARROW("skill:DribbleToGoal:direction", theFieldBall.interceptedEndPositionOnField.x(), theFieldBall.interceptedEndPositionOnField.y(), pos2.x(), pos2.y(), searchStepDrawWidth, Drawings::solidPen, ColorRGBA::cyan);
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
          auto rightIt = std::prev(it);
          if(rightIt == kickAngles.end())
            rightIt--;
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
    COMPLEX_DRAWING("skill:DribbleToGoal:direction")
    {
      Vector2f pos2 = theFieldBall.interceptedEndPositionOnField + Vector2f::polar(300.f, dribbleAngleInField);
      ARROW("skill:DribbleToGoal:direction", theFieldBall.interceptedEndPositionOnField.x(), theFieldBall.interceptedEndPositionOnField.y(), pos2.x(), pos2.y(), searchStepDrawWidth, Drawings::solidPen, ColorRGBA::red);
    }

    const Angle dribbleAngle = Angle::normalize(dribbleAngleInField - theRobotPose.rotation);
    lastDribbleAngle = dribbleAngle;
    // TODO set kickLength to something higher?
    theGoToBallAndDribbleSkill({.targetDirection = dribbleAngle,
                                .kickLength = dribbleRange,
                                .lookActiveWithBall = theFieldBall.interceptedEndPositionRelative.squaredNorm() > sqr(lookActiveMinBallDistance)});
    return;
  }

  float getDistanceToFieldBorder(const Vector2f& startPosition, const Angle& direction)
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
  }

  void calculateSectorWheel()
  {
    std::vector<ObstacleSector> obstacleSectors;
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
    {
      const Vector2f obstacleOnField = theRobotPose * obstacle.center;
      if(obstacleOnField.x() > (theFieldDimensions.xPosOpponentGroundLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
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

    wheelFiltered.addSector(lastSector->angleRange, lastSector->distance, lastSector->type);
    kickAngles = wheelFiltered.finish();

    COMPLEX_DRAWING("skill:DribbleToGoal:wheel")
    {
      DRAW_SECTOR_WHEEL("skill:DribbleToGoal:wheel", kickAngles, theFieldBall.positionOnField);
    }
  }

  using Skills::DribbleToGoalSkill::Implementation::preProcess;

  void preProcess() override
  {
    DECLARE_DEBUG_DRAWING("skill:DribbleToGoal:wheel", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:DribbleToGoal:direction", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:DribbleToGoal:step", "drawingOnField");
  }

  void reset(const DribbleToGoal&) override
  {
    lastDribbleAngle = 0_deg;
  }

private:
  struct ObstacleSector
  {
    Obstacle::Type type;
    Rangea sector; /**< The angular range relative to the ball that the obstacle blocks. */
    float distance; /**< The distance of the obstacle to the ball. */
    float x; /**< The x coordinate on field of the obstacle. */
  };

  Angle lastDribbleAngle = 0_deg;
  std::list<SectorWheel::Sector> kickAngles;
  const Vector2f bottomRightField = Vector2f(theFieldDimensions.xPosOwnGroundLine + 10.f, theFieldDimensions.yPosRightSideline + 10.f);
  const Vector2f topLeftField = Vector2f(theFieldDimensions.xPosOpponentGroundLine - 10.f, theFieldDimensions.yPosLeftSideline - 10.f);
  Rangef xMinMax = Rangef(theFieldDimensions.xPosOwnGroundLine + 10.f, theFieldDimensions.xPosOpponentGroundLine - 10.f);
  Rangef yMinMax = Rangef(theFieldDimensions.yPosRightSideline + 10.f, theFieldDimensions.yPosLeftSideline - 10.f);
  const Vector2f leftGoalPost = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal); /**< The position of the left post of the opponent's goal. */
  const Vector2f rightGoalPost = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal); /**< The position of the right post of the opponent's goal. */
};

MAKE_SKILL_IMPLEMENTATION(DribbleToGoalImpl);
