/** KEEPER PUNCH */
option(Keeper)
{
  common_transition
  {
      /*if(libCodeRelease.between(theBallModel.estimate.position.x(),200.f,600.f)
         && libCodeRelease.between(theBallModel.estimate.position.y(),0.f,1000.f)
         && libCodeRelease.between(theBallModel.estimate.velocity.x(),-1000.f, -5.f))
         goto diveLeft;
         
      if(libCodeRelease.between(theBallModel.estimate.position.x(),200.f,600.f)
         && libCodeRelease.between(theBallModel.estimate.position.y(),0.f,-1000.f)
         && libCodeRelease.between(theBallModel.estimate.velocity.x(),-1000.f, -5.f))
         goto diveRight; */
         
      if(libCodeRelease.between(theBallModel.estimate.position.x(), 20.f, 1000.f)
	//&&libCodeRelease.between(theBallModel.estimate.position.y(), -50.f, 50.f)
         && libCodeRelease.between(theBallModel.estimate.velocity.x(), -1000.f, -10.f))
      {
        //goto sumo;
	goto groundPunchLeft;
      }
  }

  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
      {
        goto alignToBall;
      }
    }
    action
    {
      LookForward();
      Stand();
    }
  }

  state(diveLeft)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
      {
        goto start;
      }
    }
    action
    {
      // ** Desactivated until the special action exist **
      //SpecialAction(SpecialActionRequest::diveLeft);
    }
  }
  
  state(diveRight)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
      {
        goto start;
      }
    }
    action
    {
      // ** Desactivated until the special action exist **
      //SpecialAction(SpecialActionRequest::diveRight);
    }
  }

  state(groundPunchLeft)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
      {
        goto start;
      }
    }
    action
    {
      
      SpecialAction(SpecialActionRequest::groundPunchLeft);
      
    }
  } 
  
  state(alignToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
      {
          if(theBallModel.estimate.velocity.y() > 0)
          {
            goto searchLeftForBall;
          }
          else if(theBallModel.estimate.velocity.y() < 0)
          {
            goto searchRightForBall;
          }
      }
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(90.f, 90.f, 90.f), Pose2f(theBallModel.estimate.position.angle(), libCodeRelease.KeeperDesiredPos));
    }
  }

  state(turnToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
      {
          if(theBallModel.estimate.velocity.y() > 0)
          {
            goto searchLeftForBall;
          }
          else if(theBallModel.estimate.velocity.y() < 0)
          {
            goto searchRightForBall;
          }
      }
      if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
      {
       goto walkToBall;
      }
    
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
      {
          if(theBallModel.estimate.velocity.y() > 0)
          {
            goto searchLeftForBall;
          }
          else if(theBallModel.estimate.velocity.y() < 0)
          {
            goto searchRightForBall;
          }
      }
      if(theBallModel.estimate.position.norm() < 500.f)
      {
        goto alignToGoal;
      }
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), libCodeRelease.KeeperDesiredPos);
    }
  }
  
  state(alignToGoal)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
      {
          if(theBallModel.estimate.position.y() > 0)
          {
            goto searchLeftForBall;
          }
          else if(theBallModel.estimate.position.y() < 0)
          {
            goto searchRightForBall;
          }
      }
      if(std::abs(libCodeRelease.angleToOppGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
      {
        goto alignBehindBall;
      }
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(libCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
    }
  }

  state(alignBehindBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
      {
          if(theBallModel.estimate.velocity.y() > 0)
          {
            goto searchLeftForBall;
          }
          else if(theBallModel.estimate.velocity.y() < 0)
          {
            goto searchRightForBall;
          }
      }
      if(libCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
         && libCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
         && std::abs(libCodeRelease.angleToOwnGoal - 180_deg) < 2_deg)
      {
        goto kick;
      }
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(libCodeRelease.angleToOppGoal, libCodeRelease.KeeperDesiredPos.x() - 150.f, libCodeRelease.KeeperDesiredPos.y() - 30.f));
    }
  }

  state(kick)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
      {
        goto start;
      }
    }
    action
    {
      LookForward();
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(libCodeRelease.angleToOppGoal, libCodeRelease.KeeperDesiredPos.x() - 160.f, libCodeRelease.KeeperDesiredPos.y() - 55.f));
    }
  }
  
  state(searchLeftForBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() < 300)
      {
        goto alignToBall;
      }
    }
    action
    {
      LookForward();
      WalkAtSpeedPercentage(Pose2f(1.f, 0.f, 0.f));
    }
  }
  
  state(searchRightForBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() < 300)
      {
        goto alignToBall;
      }
    }
    action
    {
      LookForward();
      WalkAtSpeedPercentage(Pose2f(-1.f, 0.f, 0.f));
    }
  }
}
