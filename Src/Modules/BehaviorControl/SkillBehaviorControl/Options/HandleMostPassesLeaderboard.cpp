#include "SkillBehaviorControl.h"

option((SkillBehaviorControl)HandleMostPassesLeaderboard,
  load((std::vector<Vector2f>) targetsOfInterestPlayer1,
    (std::vector<Vector2f>) targetsOfInterestPlayer2),
  vars((Pose2f)(-90_deg, 1000.f, 2000.f) finalPassingPosePlayer1, /**< Waiting pose for player1. */
    (Pose2f)(90_deg, 1000.f, -2000.f) finalPassingPosePlayer2, /**< Waiting pose for player2. */
    (Vector2f)(4000.f, 1500.f) finishPosePlayer1, /**< player 1 position in goal area to finish attempt. */
    (Vector2f)(4000.f, -1500.f) finishPosePlayer2, /**< player 2 position in goal area to finish attempt. */
    (Vector2f)(-2850.f, 2000.f) ballStartPositionPlayer1, /**< left starting position*/
    (Vector2f)(-2850.f, -2000.f) ballStartPositionPlayer2, /**< right starting position*/
    (Pose2f)(-90_deg, -2850.f, 2250.f) lookAtStartingPointPosePlayer1, /**< left walk position*/
    (Pose2f)(90_deg, -2850.f, -2250.f) lookAtStartingPointPosePlayer2, /**< right walk position*/
    (float)(750.f) dribblingKickLength, /**< Kick length for dribbling. */
    (float)(800.f) illegalAreaY, /**< width of illegal area. */
    (float)(3000.f) xThresholdDribblingDistance, /**< X distance from which the dribbling player should be more carefully. */
    (float)(3850.f) minimumKickLength, /**< Kick length for passes. */
    (int)(500) ballTimeout, /**< timeout for ball actions. */
    (int)(2000) ballSearchTimeout, /**< timeout for switching to ball search. */
    (int)(80000) reachFinalPassingPoseTime, /**< Time to reach final passing pose */
    (int)(150000) finishAttemptTime) /**< Go to finish area after this much time has passed. */)
{
  const auto selectKickType = [&](Angle kickAngle)->KickInfo::KickType
  {
    const KickInfo::Kick& kickRight = theKickInfo[KickInfo::forwardFastRight];
    const KickInfo::Kick& kickLeft = theKickInfo[KickInfo::forwardFastLeft];
    Pose2f kickPoseRight = Pose2f(kickAngle, theFieldBall.positionOnField).rotate(kickRight.rotationOffset).translate(kickRight.ballOffset);
    Pose2f kickPoseLeft = Pose2f(kickAngle, theFieldBall.positionOnField).rotate(kickLeft.rotationOffset).translate(kickLeft.ballOffset);
    const float ttrpRight = KickSelection::calcTTRP(kickPoseRight, theWalkingEngineOutput.maxSpeed);
    const float ttrpLeft = KickSelection::calcTTRP(kickPoseLeft, theWalkingEngineOutput.maxSpeed);
    return ttrpRight < ttrpLeft ? KickInfo::forwardFastRight : KickInfo::forwardFastLeft;
  };

  const auto ballInOwnHalf = [&]()->bool
  {
    return ((theRobotPose.translation.y() < 0 && theFieldBall.recentBallPositionOnField(ballTimeout, ballTimeout).y() < -illegalAreaY) ||
      (theRobotPose.translation.y() > 0 && theFieldBall.recentBallPositionOnField(ballTimeout, ballTimeout).y() > illegalAreaY));
  };

  const auto calculateWaitingPose = [&]() -> Pose2f
  {
    const GlobalTeammatesModel::TeammateEstimate* teammate = &theGlobalTeammatesModel.teammates.front();
    Vector2f kickTarget = teammate->pose.translation;
    Angle kickAngle = (kickTarget - theFieldBall.positionOnField).angle();
    const KickInfo::KickType kick = selectKickType(kickAngle);
    const Angle kickRotationOffset = theKickInfo[kick].rotationOffset;
    const Vector2f kickBallOffset = theKickInfo[kick].ballOffset;
    return Pose2f(kickAngle, theFieldBall.positionOnField).rotate(kickRotationOffset).translate(kickBallOffset);
  };

  //Check that the receiver is standing still and correctly rotated.
  const auto secondRobotReady = [&]()->bool
  {
    const GlobalTeammatesModel::TeammateEstimate* teammate = &theGlobalTeammatesModel.teammates.front();
    return (teammate->speed <= 0.f && std::abs(std::abs(teammate->pose.rotation) - 90_deg) <= 10_deg);
  };

  auto getClosestTargetPlayer1 = [&]() -> Vector2f
  {
    const GlobalTeammatesModel::TeammateEstimate* teammate = &theGlobalTeammatesModel.teammates.front();
    if (theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) > reachFinalPassingPoseTime)
      return finalPassingPosePlayer1.translation;
    for (Vector2f target : targetsOfInterestPlayer1)
    {
      if (theRobotPose.translation.x() < target.x() && teammate->pose.translation.x() <= target.x())
        return target;
    }
    return finalPassingPosePlayer1.translation;
  };

  auto getClosestTargetPlayer2 = [&]() -> Vector2f
  {
    const GlobalTeammatesModel::TeammateEstimate* teammate = &theGlobalTeammatesModel.teammates.front();
    if (theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) > reachFinalPassingPoseTime)
      return finalPassingPosePlayer2.translation;
    for (Vector2f target : targetsOfInterestPlayer2)
    {
      if (theRobotPose.translation.x() < target.x() && teammate->pose.translation.x() <= target.x())
        return target;
    }
    return finalPassingPosePlayer2.translation;
  };

  initial_state(initial)
  {
    transition
    {
      goto walkToBall;
    }
      action
    {
      LookForward();
      Stand();
    }
  }

  state(walkToBall)
  {
    transition
    {
      if (action_done)
        goto lookAtBall;
    }
      action
    {
      LookActive();
      Pose2f targetStartingPose = theGameState.playerNumber == 1 ? lookAtStartingPointPosePlayer1 : lookAtStartingPointPosePlayer2;
      WalkToPoint({.target = theRobotPose.inverse() * targetStartingPose });
    }
  }

  state(lookAtBall)
  {
    transition
    {
      if (theFieldBall.ballWasSeen() || theFieldBall.useTeamBall(ballTimeout, ballTimeout))
      {
        if (ballInOwnHalf())
          goto waitWithBall;
        else
          goto receivePass;
      }
    }
      action
    {
      Vector2f targetOnField = theGameState.playerNumber == 1 ? ballStartPositionPlayer1 : ballStartPositionPlayer2;
      LookAtPoint({.target = (Vector3f() << theRobotPose.inverse() * targetOnField, theBallSpecification.radius).finished()});
      Stand();
    }
  }

  state(passing)
  {
    transition
    {
      if (theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) > finishAttemptTime)
        goto finishAttempt;
      else if ((!theFieldBall.ballWasSeen(ballSearchTimeout) && !theFieldBall.useTeamBall(ballSearchTimeout, ballSearchTimeout)) ||
              (!theFieldBall.ballWasSeen(ballSearchTimeout) && !action_done))
        goto ballSearch;
      else if (action_done || !ballInOwnHalf())
      {
        if (theGameState.playerNumber == 1)
          goto passMovementPlayer1;
        else
          goto passMovementPlayer2;
      }
    }
      action
    {
      const GlobalTeammatesModel::TeammateEstimate * teammate = &theGlobalTeammatesModel.teammates.front();
      Vector2f kickTarget = teammate->pose.translation;
      Angle kickAngle = (kickTarget - theFieldBall.recentBallPositionOnField()).angle();
      float kickLength = std::max(minimumKickLength, (kickTarget - theFieldBall.recentBallPositionOnField()).norm());
      KickInfo::KickType kick = selectKickType(kickAngle);
      GoToBallAndKick({.targetDirection = Angle::normalize(kickAngle - theRobotPose.rotation - theKickInfo[kick].rotationOffset),
                        .kickType = kick,
                        .lookActiveWithBall = true,
                        .alignPrecisely = KickPrecision::precise,
                        .length = kickLength,
                        .preStepType = PreStepType::allowed });
    }
  }

  state(passMovementPlayer1)
  {
    transition
    {
      if (ballInOwnHalf())
        goto passing;
      if (action_done)
        goto receivePass;
    }
      action
    {
      Pose2f targetPose;
      Vector2f target = getClosestTargetPlayer1();
      targetPose = Pose2f(-90_deg, target);
      LookForward();
      WalkToPoint({.target = theRobotPose.inverse() * targetPose,
                   .disableObstacleAvoidance = true });
    }
  }

  state(passMovementPlayer2)
  {
    transition
    {
      if (ballInOwnHalf())
        goto passing;
      if (action_done)
        goto receivePass;
    }
      action
    {
      Pose2f targetPose;
      Vector2f target = getClosestTargetPlayer2();
      targetPose = Pose2f(90_deg, target);
      LookForward();
      WalkToPoint({.target = theRobotPose.inverse() * targetPose,
                   .disableObstacleAvoidance = true });
    }
  }

  state(waitWithBall)
  {
    transition
    {
      if (theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) > finishAttemptTime)
        goto finishAttempt;
      else if (secondRobotReady())
        goto passing;
    }
      action
    {
      const Pose2f targetPose = calculateWaitingPose();
      WalkToPoint({.target = theRobotPose.inverse() * targetPose,
                   .disableEnergySavingWalk = true,
                   .disableObstacleAvoidance = true,
                   .disableAvoidFieldBorder = true,
                   .targetOfInterest = theFieldBall.recentBallPositionRelative()});
      const GlobalTeammatesModel::TeammateEstimate* teammate = &theGlobalTeammatesModel.teammates.front();
      Vector2f targetSecondPlayer = teammate->pose.translation;
      LookAtPoint({.target = (Vector3f() << theRobotPose.inverse() * targetSecondPlayer, 0.f).finished()});
    }
  }

  state(ballSearch)
  {
    transition
    {
      if (theFieldBall.ballWasSeen() && ballInOwnHalf())
        goto waitWithBall;
      else if (theFieldBall.useTeamBall() && !ballInOwnHalf())
      {
        if (theGameState.playerNumber == 1)
          goto passMovementPlayer1;
        else
          goto passMovementPlayer2;
      }
    }
      action
    {
      DemoSearchForBall();
    }
  }

  state(receivePass)
  {
    transition
    {
      if (!theFieldBall.ballWasSeen(ballSearchTimeout) && !theFieldBall.useTeamBall(ballSearchTimeout, ballSearchTimeout))
        goto ballSearch;
      else if ((ballInOwnHalf() && (theFieldBall.velocityRelative).squaredNorm() <= 0.f) ||
              ((theGameState.playerNumber == 1 && theFieldBall.recentBallPositionOnField(ballTimeout, ballTimeout).y() > theRobotPose.translation.y()) ||
               (theGameState.playerNumber == 2 && theFieldBall.recentBallPositionOnField(ballTimeout, ballTimeout).y() < theRobotPose.translation.y())))
        goto waitWithBall;
    }
      action
    {
      const GlobalTeammatesModel::TeammateEstimate * teammate = &theGlobalTeammatesModel.teammates.front();
      const Angle targetAngle = (theRobotPose.inverse() * teammate->pose).translation.angle();
      TurnAngle({.angle = targetAngle,
                  .margin = 5_deg });
      LookAtBall();
    }
  }

  state(finishAttempt)
  {
    transition
    {
      if (!ballInOwnHalf() || theGameState.isFinished())
        goto stand;
    }
      action
    {
      Vector2f dribblingTarget = theGameState.playerNumber == 1 ?
                                 finishPosePlayer1 : finishPosePlayer2;
      Angle dribblingAngle = (dribblingTarget - theFieldBall.positionOnField).angle();
      if (theFieldBall.positionOnField.x() >= xThresholdDribblingDistance)
        dribblingKickLength = 400.f;
      GoToBallAndDribble({.targetDirection = Angle::normalize(dribblingAngle - theRobotPose.rotation),
                          .kickLength = dribblingKickLength});
    }
  }

  state(stand)
  {
    transition
    {
      if (ballInOwnHalf())
        goto finishAttempt;
    }
      action
    {
      Stand();
      LookActive();
    }
  }
}
