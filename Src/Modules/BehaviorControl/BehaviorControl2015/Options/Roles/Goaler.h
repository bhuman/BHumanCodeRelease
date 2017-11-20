/** A test goaler */



option(Goaler)
{
  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
        goto waitForBall;
    }
    action
    {
      LookForward();
      Stand();
    }
  }

state(waitForBall)
  {
    transition
    {
     // if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        //goto searchForBall;
	//Stand();
	
      if(libCodeRelease.between(theBallModel.estimate.position.x(),0.f,500.f)
	&& libCodeRelease.between(theBallModel.estimate.position.y(),0.f,500.f))
	//&& libCodeRelease.between(theBallModel.estimate.velocity.x(),1.f,1000.f)
	//&& libCodeRelease.between(theBallModel.estimate.velocity.y(),1.f,1000.f))
        goto stopBall;
	


	//if((theBallModel.estimate.position.norm() < 500.f)) 
	//&& libCodeRelease.between(theBallModel.estimate.velocity.x(),1.f,1000.f))
	 
		
    }
    action
    {
      LookForward();
      Stand();
    }
  }



state(stopBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        goto waitForBall;
	//Stand();
      /*if(libCodeRelease.between(theBallModel.estimate.position.x(),40.f,400.f)
	&& libCodeRelease.between(theBallModel.estimate.position.y(),40.f,400.f))*/
	
    }
    action
    {
        
	SpecialAction(SpecialActionRequest::dive);
    }
  }




/**
state(waitForBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(libCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
          && libCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
          && std::abs(libCodeRelease.angleToGoal) < 2_deg)
        goto kick;
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
    }
  }
**/



/**
  state(turnToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
       goto walkToBall;
	   
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }

*/
/**
  state(walkToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(theBallModel.estimate.position.norm() < 200.f)
        //goto alignToGoal;
	goto alignBehindBall;
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);
    }
  }

*/

/**
  state(alignToGoal)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(libCodeRelease.angleToGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
        goto alignBehindBall;
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
    }
  }


  state(alignBehindBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(libCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
          && libCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
          && std::abs(libCodeRelease.angleToGoal) < 2_deg)
        goto kick;
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
    }
  }

*/

/**
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
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
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
      LookForward();
      //WalkAtSpeedPercentage(Pose2f(1.f, 0.f, 0.f));
    }
  }
*/
}
