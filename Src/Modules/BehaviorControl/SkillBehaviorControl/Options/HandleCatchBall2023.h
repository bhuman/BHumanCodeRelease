option(HandleCatchBall2023)
{
  initial_state(noCatch)
  {
    transition
    {
      if(theFrameInfo.getTimeSince(theFieldBall.lastInterceptBall) < 300 && !theFieldBall.interceptBall && theFieldBall.positionRelative.x() > 0.f)
        goto backWalking;
    }
  }

  state(backWalking)
  {
    transition
    {
      if(state_time > 300 || theFieldBall.interceptBall)
        goto noCatch;
    }
    action
    {
      theWalkAtRelativeSpeedSkill({ 0.f, -0.3f, 0.f });
      theLookAtBallSkill();
    }
  }
}
