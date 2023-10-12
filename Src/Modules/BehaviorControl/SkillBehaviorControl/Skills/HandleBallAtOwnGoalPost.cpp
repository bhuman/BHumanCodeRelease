/**
 * @file HandleBallAtOwnGoalPost.cpp
 *
 * This file defines an implementation of the HandleBallAtOwnGoalPost skill.
 *
 * @author Jesse Richter-Klug
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Debugging/DebugDrawings.h"
#include "Math/BHMath.h"
#include "Math/Geometry.h"
#include "Math/Range.h"
#include "RobotParts/Arms.h"
#include <array>
#include <limits>

SKILL_IMPLEMENTATION(HandleBallAtOwnGoalPostImpl,
{,
  IMPLEMENTS(HandleBallAtOwnGoalPost),
  CALLS(GoToBallAndKick),
  CALLS(KeyFrameSingleArm),
  CALLS(LookActive),
  CALLS(WalkToPoint),
  CALLS(Zweikampf),
  REQUIRES(BallSpecification),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(KickInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  REQUIRES(WalkingEngineOutput),
  DEFINES_PARAMETERS(
  {,
    (float)(500.f) radiusHandlingArea, /**< The radius of the area around the goal posts which shall receive special handling. */
    (float)(1.2f) hysteresisMultiplierHandlingArea, /**< If this card was previously active, the radius is multiplied by this factor. */
    (float)(100.f) ballGoalPostTangentOffset, /**< This amount of space should fit between the goal post and the ball. */
    (float)(50.f) robotGoalPostTangentOffset, /**< This amount of space should fit between the goal post and the robot. */
    (float)(500.f) hysteresisDecisionPenalty, /**< Changing the decision of the previous frame results in this penalty to the time. */
    (std::vector<KickInfo::KickType>)({KickInfo::walkForwardsLeft, KickInfo::walkForwardsRight,
                                       KickInfo::walkForwardsLeftLong, KickInfo::walkForwardsRightLong,
                                       KickInfo::walkTurnLeftFootToRight, KickInfo::walkTurnRightFootToLeft}) availableKicks, /**< The kicks that may be selected. */
  }),
});

class HandleBallAtOwnGoalPostImpl : public HandleBallAtOwnGoalPostImplBase
{
  void execute(const HandleBallAtOwnGoalPost& p) override
  {
    const bool isLeft = theFieldBall.positionOnField.y() > 0.f;
    const Vector2f usedGoalPost(std::min(theFieldDimensions.xPosOwnGoalPost, theFieldBall.positionOnField.x() - theFieldDimensions.goalPostRadius), isLeft ? theFieldDimensions.yPosLeftGoal : theFieldDimensions.yPosRightGoal);

    const Vector2f ballInGoalPost = theFieldBall.positionOnField - usedGoalPost;
    const float distBallGoalPost = ballInGoalPost.norm();
    const float minBallGoalPostDistance = theFieldDimensions.goalPostRadius + theBallSpecification.radius + ballGoalPostTangentOffset;
    const Angle ballGoalPostTangentAngleOffset = std::asin(std::min(1.f, minBallGoalPostDistance / distBallGoalPost));
    const Angle ballGoalPostTangentAngle = Angle::normalize((-ballInGoalPost).angle() + (isLeft ? -ballGoalPostTangentAngleOffset : ballGoalPostTangentAngleOffset));
    ASSERT(!std::isnan(float(ballGoalPostTangentAngle)));

    SectorWheel blueprintWheel;
    blueprintWheel.begin(theFieldBall.positionOnField);
    blueprintWheel.addSector(isLeft ? Rangea(ballGoalPostTangentAngle, 0.f) : Rangea(0.f, ballGoalPostTangentAngle), 0.f, SectorWheel::Sector::erased);
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
    {
      const Vector2f obstacleOnField = theRobotPose * obstacle.center;

      // There may be a lot of "low-quality" obstacles behind on the border strip.
      if(obstacleOnField.x() < theFieldDimensions.xPosOwnGroundLine + 300.f)
        continue;

      const float width = (obstacle.left - obstacle.right).norm() + 4.f * theBallSpecification.radius;
      const float distance = std::sqrt(std::max((obstacleOnField - theFieldBall.interceptedEndPositionOnField).squaredNorm() - sqr(width / 2.f), 1.f));
      if(distance < theBallSpecification.radius)
        continue;

      const float radius = std::atan(width / (2.f * distance));
      const Angle direction = (obstacleOnField - theFieldBall.interceptedEndPositionOnField).angle();
      const Rangea angleRange(direction - radius, direction + radius);

      blueprintWheel.addSector(angleRange, distance, SectorWheel::Sector::obstacle);
    }

    COMPLEX_DRAWING("skill:HandleBallAtOwnGoalPost:blueprintWheel")
    {
      SectorWheel wheel = blueprintWheel;
      auto sectors = wheel.finish();
      DRAW_SECTOR_WHEEL("skill:HandleBallAtOwnGoalPost:blueprintWheel", sectors, theFieldBall.positionOnField);
    }

    KickInfo::KickType bestKick = KickInfo::numOfKickTypes;
    Pose2f bestKickPoseRelative;
    float bestTTRP = std::numeric_limits<float>::max();

    const float minRobotGoalPostDistance = theFieldDimensions.goalPostRadius + robotGoalPostTangentOffset;
    const Angle robotGoalPostTangentAngleOffset = std::asin(minRobotGoalPostDistance / std::max(minRobotGoalPostDistance, distBallGoalPost));
    const Angle goalPostAngle0 = Angle::normalize(ballInGoalPost.angle() - robotGoalPostTangentAngleOffset);
    const Angle goalPostAngle1 = Angle::normalize(ballInGoalPost.angle() + robotGoalPostTangentAngleOffset);

    for(KickInfo::KickType kickType : availableKicks)
    {
      SectorWheel wheel = blueprintWheel;

      const ContourAngleOffsets& angleOffsets = contourAngleOffsets[kickType];
      wheel.addSector(Rangea(Angle::normalize(goalPostAngle0 + angleOffsets.rightAngleOffset),
                             Angle::normalize(goalPostAngle1 + angleOffsets.leftAngleOffset)),
                      0.f, SectorWheel::Sector::erased);
      auto sectors = wheel.finish();

      if(kickType == drawKickType)
        DRAW_SECTOR_WHEEL("skill:HandleBallAtOwnGoalPost:kickWheel", sectors, theFieldBall.positionOnField);

      for(const SectorWheel::Sector& sector : sectors)
      {
        if(sector.type != SectorWheel::Sector::free)
          continue;

        const Pose2f kickPoseRelative = theRobotPose.inverse() * KickSelection::calcOptimalKickPoseForTargetAngleRange(sector.angleRange, theRobotPose, theFieldBall.positionOnField, theKickInfo[kickType].ballOffset, theKickInfo[kickType].rotationOffset);
        const float ttrp = KickSelection::calcTTRP(kickPoseRelative, theWalkingEngineOutput.maxSpeed) + theKickInfo[kickType].executionTime + (kickType == lastKickType ? 0.f : hysteresisDecisionPenalty);
        if(ttrp < bestTTRP)
        {
          bestKick = kickType;
          bestKickPoseRelative = kickPoseRelative;
          bestTTRP = ttrp;
        }
      }
    }
    lastKickType = bestKick;
    setArm(Arms::left, usedGoalPost);
    setArm(Arms::right, usedGoalPost);
    if(bestKick != KickInfo::numOfKickTypes)
    {
      theGoToBallAndKickSkill({.targetDirection = Angle::normalize(bestKickPoseRelative.rotation - theKickInfo[bestKick].rotationOffset),
                               .kickType = bestKick});
      p.setState("clear");
    }
    else
    {
      theZweikampfSkill();
    }
  }

  void reset(const HandleBallAtOwnGoalPost&) override
  {
    armWasBack[Arms::left] = armWasBack[Arms::right] = false;
    lastKickType = KickInfo::numOfKickTypes;
  }

  void preProcess() override {}

  void preProcess(const HandleBallAtOwnGoalPost&) override
  {
    DECLARE_DEBUG_DRAWING("skill:HandleBallAtOwnGoalPost:blueprintWheel", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:HandleBallAtOwnGoalPost:kickWheel", "drawingOnField");

    drawKickType = KickInfo::numOfKickTypes;
    MODIFY("skill:HandleBallAtOwnGoalPost:drawKickType", drawKickType);
  }

  void setArm(Arms::Arm arm, const Vector2f& goalPost)
  {
    if((armWasBack[arm] = (theRobotPose.translation - goalPost).squaredNorm() < sqr(armWasBack[arm] ? radiusHandlingArea * hysteresisMultiplierHandlingArea : radiusHandlingArea) &&
                          (arm == Arms::left ? 1.f : -1.f) * (theRobotPose.inverse() * goalPost).angle() > (armWasBack[arm] ? -20_deg : 0_deg)))
      theKeyFrameSingleArmSkill({.motion = ArmKeyFrameRequest::back,
                                 .arm = arm});
  }

  std::array<bool, Arms::numOfArms> armWasBack; /**< Whether the arm was back in the previous frame. */
  KickInfo::KickType drawKickType; /**< The kick type for which the sector wheel should be drawn. */
  KickInfo::KickType lastKickType; /**< The kick type that has been selected in the previous frame. */

public:
  HandleBallAtOwnGoalPostImpl()
  {
    // This, of course, belongs into a configuration file.
    std::vector<Vector2f> robotShape;
    robotShape.emplace_back(-30.f, 120.f);
    robotShape.emplace_back(50.f, 120.f);
    robotShape.emplace_back(110.f, 80.f);
    robotShape.emplace_back(110.f, -80.f);
    robotShape.emplace_back(50.f, -120.f);
    robotShape.emplace_back(-30.f, -120.f);

    FOREACH_ENUM(KickInfo::KickType, kickType)
    {
      const Vector2f& ballOffset = theKickInfo[kickType].ballOffset;
      const Angle baseAngle = ballOffset.angle();
      Angle max = -pi, min = pi;
      for(const Vector2f& contactPoint : robotShape)
      {
        const Angle angle = (ballOffset + contactPoint).angle();
        const Angle angleInBase = Angle::normalize(angle - baseAngle);
        if(angleInBase > max)
          max = angleInBase;
        if(angleInBase < min)
          min = angleInBase;
      }
      contourAngleOffsets[kickType].leftAngleOffset = -Angle::normalize(pi - max + baseAngle + theKickInfo[kickType].rotationOffset);
      contourAngleOffsets[kickType].rightAngleOffset = -Angle::normalize(pi - min + baseAngle + theKickInfo[kickType].rotationOffset);
    }
  }

private:
  struct ContourAngleOffsets
  {
    Angle leftAngleOffset; /**< The offset to the left goal post tangent to avoid standing in a goal post. */
    Angle rightAngleOffset; /**< The offset to the right goal post tangent to avoid standing in a goal post. */
  };

  std::array<ContourAngleOffsets, KickInfo::numOfKickTypes> contourAngleOffsets; /**< The angle offsets per kick to avoid standing in a goal post. */
};

MAKE_SKILL_IMPLEMENTATION(HandleBallAtOwnGoalPostImpl);
