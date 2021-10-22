/**
 * @file KickDeviationTestCard.cpp
 *
 * This file implements a card that handles behavior test InWalkKicks.
 *
 * @author Philip Reichenberg
 */

#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(KickDeviationTestCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(LookLeftAndRight),
  CALLS(LookActive),
  CALLS(LookAtBall),
  CALLS(Stand),
  CALLS(WalkToPoint),
  CALLS(WalkAtRelativeSpeed),
  REQUIRES(FieldBall),
  REQUIRES(FrameInfo),
  REQUIRES(KeyStates),
  REQUIRES(KickInfo),
  REQUIRES(MotionInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(OdometryData),
  REQUIRES(RobotInfo),
  LOADS_PARAMETERS(
  {,
    (int) lookAtStartTime,
    (float) lookLeftAndRightMinDistanceToBall,
    (int) miscActiveTimeCheck,
    (KickInfo::KickType) kick,
    (float) minDistanceToObstacle,
  }),
});

class KickDeviationTestCard : public KickDeviationTestCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return false;
  }

  void reset() override
  {
    startTimestamp = 0;
    lastActiveTimestamp = 0;
    hasTarget = false;
    finished = false;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::unknown);

    if(finished)
    {
      if((kick == KickInfo::walkSidewardsLeftFootToLeft || kick == KickInfo::walkSidewardsRightFootToRight) && theFrameInfo.getTimeSince(finishedTimestamp) < 1000)
      {
        float sign = theKickInfo[kick].rotationOffset > 0 ? -1.f : 1.f;
        theWalkAtRelativeSpeedSkill(Pose2f(sign, 0.f, 0.f));
      }
      else
        theStandSkill();
      theLookAtBallSkill();
      if((theFieldBall.positionRelative - lastBallPosition).norm() > 10.f)
        ballBigMovementTimestamp = theFrameInfo.time;
      lastBallPosition = theFieldBall.positionRelative;
      if(theFrameInfo.getTimeSince(ballBigMovementTimestamp) > 1000)
      {
        Pose2f odoDif = odoAtStart.inverse() * theOdometryData;
        Vector2f ballInOld = odoDif * theFieldBall.positionRelative;
        float length = (ballInOld - ballStartPosition).norm();
        Angle kickAngle = (ballInOld - ballStartPosition).angle();
        OUTPUT_TEXT(length << " " << kickAngle.toDegrees());
      }
      return;
    }
    const bool pressed = theKeyStates.pressed[KeyStates::lFootLeft] || theKeyStates.pressed[KeyStates::lFootRight];
    if(!lastBumperPress && pressed)
      SystemCall::say(TypeRegistry::getEnumName(kick));
    lastBumperPress = pressed;

    if(theFrameInfo.time > lastActiveTimestamp + miscActiveTimeCheck)
      startTimestamp = theFrameInfo.time;
    lastActiveTimestamp = theFrameInfo.time;
    if(theFrameInfo.getTimeSince(startTimestamp) < lookAtStartTime || theFieldBall.timeSinceBallDisappeared > 200)
    {
      theLookLeftAndRightSkill();
      theStandSkill(false);
      return;
    }
    if(theFieldBall.positionRelative.norm() > lookLeftAndRightMinDistanceToBall)
      theLookLeftAndRightSkill();

    if(theFieldBall.timeSinceBallDisappeared > 1000)
    {
      theLookLeftAndRightSkill();
      return;
    }

    hasTarget = false;
    for(const Obstacle& ob : theObstacleModel.obstacles)
    {
      if(ob.type == Obstacle::goalpost)
        continue;
      if(ob.center.norm() < minDistanceToObstacle)
        continue;
      if(std::abs((ob.center - theFieldBall.positionRelative).angle()) < 20_deg)
      {
        targetOB = ob;
        hasTarget = true;
      }
    }
    if(hasTarget && theFieldBall.positionRelative.norm() < lookLeftAndRightMinDistanceToBall)
      theGoToBallAndKickSkill((targetOB.center - theFieldBall.positionRelative).angle(), kick, true);
    else
      theWalkToPointSkill(Pose2f(0_deg, theFieldBall.positionRelative + theFieldBall.positionRelative.normalized(-lookLeftAndRightMinDistanceToBall + 200.f)));
    if(!hasTarget && theFieldBall.positionRelative.norm() < lookLeftAndRightMinDistanceToBall)
      theLookLeftAndRightSkill();

    if(theMotionInfo.isWalkPhaseInWalkKick && !finished)
    {
      finished = true;
      odoAtStart = theOdometryData;
      kickDirection = (targetOB.center - theFieldBall.positionRelative).angle();
      ballStartPosition = theFieldBall.positionRelative;
      ballBigMovementTimestamp = theFrameInfo.time;
      finishedTimestamp = theFrameInfo.time;
      lastBallPosition = theFieldBall.positionRelative;
    }
  }

private:
  unsigned startTimestamp = 0;
  unsigned lastActiveTimestamp = 0;
  Obstacle targetOB;
  bool hasTarget = false;
  bool lastBumperPress = false;
  bool finished = false;
  Vector2f ballStartPosition;
  Vector2f lastBallPosition;
  unsigned ballBigMovementTimestamp = 0;
  unsigned finishedTimestamp = 0;
  Angle kickDirection;
  OdometryData odoAtStart;
};

MAKE_CARD(KickDeviationTestCard);
