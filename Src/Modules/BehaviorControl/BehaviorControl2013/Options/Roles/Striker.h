/** A test striker without common decision */
option(Striker)
{
  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  state(turnToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(std::abs(theBallModel.estimate.position.angle()) < fromDegrees(5.f))
        goto walkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2D(50.f, 50.f, 50.f), Pose2D(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }

  state(walkToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(theBallModel.estimate.position.abs() < 500.f)
        goto alignToGoal;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2D(50.f, 50.f, 50.f), theBallModel.estimate.position);
    }
  }

  state(alignToGoal)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(std::abs(libCodeRelease.angleToGoal) < fromDegrees(10.f) && std::abs(theBallModel.estimate.position.y) < 100.f)
        goto alignBehindBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2D(100.f, 100.f, 100.f), Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 400.f, theBallModel.estimate.position.y));
    }
  }

  state(alignBehindBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(libCodeRelease.between(theBallModel.estimate.position.y, 20.f, 50.f)
        && libCodeRelease.between(theBallModel.estimate.position.x, 140.f, 170.f)
        && std::abs(libCodeRelease.angleToGoal) < fromDegrees(2.f))
        goto kick;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2D(80.f, 80.f, 80.f), Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 150.f, theBallModel.estimate.position.y - 30.f));
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
      theHeadControlMode = HeadControl::lookForward;
      InWalkKick(WalkRequest::left, Pose2D(libCodeRelease.angleToGoal, theBallModel.estimate.position.x - 160.f, theBallModel.estimate.position.y - 55.f));
    }
  }
  
  state(searchForBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() < 300)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;    
      WalkAtSpeedPercentage(Pose2D(1.f, 0.f, 0.f));
    }
  }
}
