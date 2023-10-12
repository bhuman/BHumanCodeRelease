option(HandleCatchBall)
{
  initial_state(noCatch)
  {
    transition
    {
      if(theFieldBall.interceptBall)
        goto catching;
    }
  }

  state(catching)
  {
    transition
    {
      if(!theFieldBall.interceptBall ||
         theInterceptBallSkill.isDone())
        goto noCatch;
    }
    action
    {
      unsigned interceptionMethods = bit(Interception::stand) | bit(Interception::walk);
      // Removed for 2021 competitions:
      // if(!theDamageConfigurationBody.noFieldGenuflect && theFrameInfo.getTimeSince(_context.behaviorStart) > genuflectDelay && theMotionInfo.executedPhase == MotionPhase::stand)
      //   interceptionMethods |= bit(Interception::genuflectStandDefender);
      theInterceptBallSkill({.interceptionMethods = interceptionMethods});
    }
  }
}
