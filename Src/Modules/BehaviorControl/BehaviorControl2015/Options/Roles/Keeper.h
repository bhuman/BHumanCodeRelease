option(Keeper)
{
  common_transition
  {
      /*if(LibTactic.between(theBallModel.estimate.position.x(),200.f,600.f)
         && LibTactic.between(theBallModel.estimate.position.y(),0.f,1000.f)
         && LibTactic.between(theBallModel.estimate.velocity.x(),5.f,1000.f))
         goto diveLeft;
         
      if(LibTactic.between(theBallModel.estimate.position.x(),200.f,600.f)
         && LibTactic.between(theBallModel.estimate.position.y(),0.f,-1000.f)
         && LibTactic.between(theBallModel.estimate.velocity.x(),5.f,1000.f))
         goto diveRight; */

      if(LibTactic.between(theBallModel.estimate.position.x(), 200.f, 1000.f)
         && LibTactic.between(theBallModel.estimate.velocity.x(), -1000.f, -5.f))
      {
        goto sumo;
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
      if(LibTactic.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
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
      if(LibTactic.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
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

  state(sumo)
  {
    transition
    {
      if(LibTactic.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
      {
        goto start;
      }
    }
    action
    {

      SpecialAction(SpecialActionRequest::sumo);
    }
  }

  state(alignToBall)
  {
    transition
    {
      if(LibTactic.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
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
      WalkToTarget(Pose2f(90.f, 90.f, 90.f), Pose2f(theBallModel.estimate.position.angle(), LibTactic.DesiredPos));
    }
  }

  state(turnToBall)
  {
    transition
    {
      if(LibTactic.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
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
      if(LibTactic.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
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
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), LibTactic.DesiredPos);
    }
  }

  state(alignToGoal)
  {
    transition
    {
      if(LibTactic.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
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
      if(std::abs(LibTactic.angleToOppGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
      {
        goto alignBehindBall;
      }
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(LibTactic.angleToOppGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
    }
  }

  state(alignBehindBall)
  {
    transition
    {
      if(LibTactic.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
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
      if(LibTactic.between(theBallModel.estimate.position.y(), 20.f, 50.f)
         && LibTactic.between(theBallModel.estimate.position.x(), 140.f, 170.f)
         && std::abs(LibTactic.angleToOwnGoal - 180_deg) < 2_deg)
      {
        goto kick;
      }
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(LibTactic.angleToOppGoal, LibTactic.DesiredPos.x() - 150.f, LibTactic.DesiredPos.y() - 30.f));
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
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(LibTactic.angleToOppGoal, LibTactic.DesiredPos.x() - 160.f, LibTactic.DesiredPos.y() - 55.f));
    }
  }
  
  state(searchLeftForBall)
  {
    transition
    {
      if(LibTactic.timeSinceBallWasSeen() < 300)
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
      if(LibTactic.timeSinceBallWasSeen() < 300)
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
