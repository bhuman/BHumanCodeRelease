/** A test Defender  */
option(Defender)
{
  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
        goto searchForBall;
    }
    action
    {
      LookForward();
      Stand();
    }
  }

  state(alignToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(90.f, 90.f, 90.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }


  state(turnToBallFar)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(theBallModel.estimate.position.angle()) < 5_deg 
          && theBallModel.estimate.position.x() < 100.f
           && theBallModel.estimate.position.y() < 100.f)
        goto walkToBall;
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }



  state(walkToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(theBallModel.estimate.position.norm() < 100.f)
        goto alignBehindBall;
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);
    }
  }

/*  state(alignToGoal)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(libCodeRelease.angleToOppGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
        goto alignBehindBall;
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(libCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
    }
  }
*/
  state(alignBehindBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(libCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
          && libCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
          && std::abs(libCodeRelease.angleToOppGoal) < 2_deg)
        goto kick;
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(libCodeRelease.angleToOppGoal, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
    }
  }

  state(kick)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
    }
    action
    {
      LookForward();
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(libCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
    }
  }
  
  state(searchForBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() < 300
          &&theBallModel.estimate.position.x() >200.f
           &&theBallModel.estimate.position.y() > 200.f)
        goto turnToBallFar;
      else if (theBallModel.estimate.position.x() < 5.f
           &&theBallModel.estimate.position.y() < 5.f)
        goto blockAttack; 
    }
    action
    {
      LookForward();
      WalkAtSpeedPercentage(Pose2f(1.f, 0.f, 0.f));
    }
  }


state(blockAttack)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(theBallModel.estimate.position.norm() < 200.f)
        goto walkToBall;
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), theBallModel.estimate.position.y() - 50.f
                                                    ,theBallModel.estimate.position.y() - 50.f));
    }
  }
}
