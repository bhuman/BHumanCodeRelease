#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) PenaltyTaker,
       vars((std::array<unsigned, 3>)({}) obstacleCellTimestamps)) /**< The timestamps when an obstacles was in each cell. */
{
  const float obstacleCenterThreshold = 180.f; // Obstacles closer than this to the goal center are considered to be in the goal center.
  const int obstacleCenterPriority = 500; // If an obstacle in the goal center has been seen within this duration, all other obstacles are ignored.
  const int obstacleMaxAge = 1000; // Obstacles older than this are ignored.

  // As long as there is an obstacle in the center of the goal, percepts in the corners will be ignored.
  const bool hasCenterObstacle = theFrameInfo.getTimeSince(obstacleCellTimestamps[1]) < obstacleCenterPriority;
  for(auto& percept : theObstaclesFieldPercept.obstacles)
  {
    // The percept is only accepted if it is in a reasonable position.
    const Vector2f perceptOnField = theRobotPose * percept.center;
    if(perceptOnField.x() < theFieldDimensions.xPosOpponentGoalArea - 300.f
       || perceptOnField.x() > theFieldDimensions.xPosOpponentGoalLine + 100.f
       || std::abs(perceptOnField.y()) > (hasCenterObstacle ? obstacleCenterThreshold : theFieldDimensions.yPosLeftGoal - 250.f))
      continue;

    // Due to the goal line, it can happen that a percept seems very wide and spans the complete goal. In that case, the obstacle is ignored.
    if((percept.left - percept.right).squaredNorm() > sqr(600.f))
      continue;

    if(std::abs(perceptOnField.y()) <= obstacleCenterThreshold)
      obstacleCellTimestamps[1] = theFrameInfo.time;
    else if(perceptOnField.y() > 0)
      obstacleCellTimestamps[0] = theFrameInfo.time;
    else
      obstacleCellTimestamps[2] = theFrameInfo.time;
  }

  auto isPenaltyShootoutObstacleInGoal = [&](const bool left)
  {
    return theFrameInfo.getTimeSince(obstacleCellTimestamps[left ? 0 : 2]) < obstacleMaxAge;
  };

  auto kickLeft = [&]
  {
    if(theBehaviorParameters.penaltyStrikerUseObstacles)
    {
      if(isPenaltyShootoutObstacleInGoal(true))
        return false;
      else if(isPenaltyShootoutObstacleInGoal(false))
        return true;
    }
    return ((theFrameInfo.time >> 4) & 1) != 0;
  };

  if(theGameState.isPushingFreeKick())
    Say({.text = "Taking Penalty"});

  const int timeSincePenaltyShootoutStarted = theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted);
  const int timeUntilPenaltyShootoutEnds = -theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds);

  common_transition
  {
    if(timeSincePenaltyShootoutStarted > 5000 && !theFieldBall.ballWasSeen(5000))
      goto goBehindPenaltyMark;
  }

  initial_state(initial)
  {
    transition
    {
      if(std::min(timeSincePenaltyShootoutStarted, state_time) > 3000 || timeUntilPenaltyShootoutEnds <= 10000)
      {
        if(kickLeft())
          goto goToBallAndKickLeft;
        else
          goto goToBallAndKickRight;
      }
    }
    action
    {
      LookLeftAndRight({.maxPan = 20_deg,
                        .tilt = 5.7_deg,
                        .speed = 30_deg});
      Stand();
    }
  }

  state(goToBallAndKickLeft)
  {
    action
    {
      const Vector2f goalPostOnField(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
      const Vector2f ballPositionOnField = theRobotPose * theBallModel.estimate.position;
      const Angle angle = (goalPostOnField - ballPositionOnField).angle() -
                          theBehaviorParameters.penaltyStrikerAngleToLeftPostOffset;

      // The right foots kick direction deviates to the right -> more to the middle of the goal. The left to the left -> goal post. We do not want to hit the goal post!
      KickInfo::KickType kickType = KickInfo::forwardFastRightLong;
      if(theDamageConfigurationBody.sides[Legs::left].weakLeg && !theDamageConfigurationBody.sides[Legs::right].weakLeg)
        kickType = KickInfo::forwardFastLeftLong;

      if(theGameState.isPenaltyKick())
        GoToBallAndKick({.targetDirection = Angle::normalize(angle - theRobotPose.rotation),
                         .kickType = kickType,
                         .alignPrecisely = KickPrecision::precise,
                         .speed = {theBehaviorParameters.penaltyStrikerWalkSpeed, theBehaviorParameters.penaltyStrikerWalkSpeed, theBehaviorParameters.penaltyStrikerWalkSpeed},
                         .reduceWalkSpeedType = ReduceWalkSpeedType::slow});
      else
      {
        Pose2f kickPoseOnField(angle, ballPositionOnField);
        kickPoseOnField.rotate(theKickInfo.kicks[kickType].rotationOffset);
        kickPoseOnField.translate(theKickInfo.kicks[kickType].ballOffset);
        PenaltyStrikerGoToBallAndKick({.kickPose = theRobotPose.inverse() * kickPoseOnField,
                                       .kickType = kickType,
                                       .walkSpeed = theBehaviorParameters.penaltyStrikerWalkSpeed});
      }
    }
  }

  state(goToBallAndKickRight)
  {
    action
    {
      const Vector2f goalPostOnField(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
      const Vector2f ballPositionOnField = theRobotPose * theBallModel.estimate.position;
      const Angle angle = (goalPostOnField - ballPositionOnField).angle() +
                          theBehaviorParameters.penaltyStrikerAngleToRightPostOffset;

      // The left foots kick direction deviates to the left -> more to the middle of the goal. The right to the right -> goal post. We do not want to hit the goal post!
      KickInfo::KickType kickType = KickInfo::forwardFastLeftLong;
      if(theDamageConfigurationBody.sides[Legs::right].weakLeg && !theDamageConfigurationBody.sides[Legs::left].weakLeg)
        kickType = KickInfo::forwardFastRightLong;

      if(theGameState.isPenaltyKick())
        GoToBallAndKick({.targetDirection = Angle::normalize(angle - theRobotPose.rotation),
                         .kickType = kickType,
                         .alignPrecisely = KickPrecision::precise,
                         .speed = {theBehaviorParameters.penaltyStrikerWalkSpeed, theBehaviorParameters.penaltyStrikerWalkSpeed, theBehaviorParameters.penaltyStrikerWalkSpeed},
                         .reduceWalkSpeedType = ReduceWalkSpeedType::slow });
      else
      {
        Pose2f kickPoseOnField(angle, ballPositionOnField);
        kickPoseOnField.rotate(theKickInfo.kicks[kickType].rotationOffset);
        kickPoseOnField.translate(theKickInfo.kicks[kickType].ballOffset);
        PenaltyStrikerGoToBallAndKick({.kickPose = theRobotPose.inverse() * kickPoseOnField,
                                       .kickType = kickType,
                                       .walkSpeed = theBehaviorParameters.penaltyStrikerWalkSpeed});
      }
    }
  }

  state(goBehindPenaltyMark)
  {
    transition
    {
      if(theFieldBall.ballWasSeen())
      {
        if(theBallModel.estimate.position.squaredNorm() < sqr(500.f))
        {
          if(kickLeft())
            goto goToBallAndKickLeft;
          else
            goto goToBallAndKickRight;
        }
        else
          goto initial;
      }
    }
    action
    {
      LookActive({.withBall = true,
                  .onlyOwnBall = true});
      const Vector2f target = theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOpponentPenaltyMark - 300.f, 0.f);
      WalkToPoint({.target = {-theRobotPose.rotation, target},
                   .speed = {theBehaviorParameters.penaltyStrikerWalkSpeed, theBehaviorParameters.penaltyStrikerWalkSpeed, theBehaviorParameters.penaltyStrikerWalkSpeed},
                   .rough = true,
                   .disableObstacleAvoidance = true});
    }
  }
}
