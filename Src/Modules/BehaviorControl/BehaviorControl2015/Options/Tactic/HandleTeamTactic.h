option(HandleTeamTactic)
{
  initial_state(ChooseRole)
  {
    transition
    {
      if(theRobotInfo.number == 1)
        goto PlayKeeper;
      else
        goto PlayStriker;
    }
    action
    {
      LookForward();
    }
  }
  
  state(PlayKeeper)
  {
    action
    {
      LookForward();
      Keeper();
    }
  }
  
  state(PlayStriker)
  {
    transition
    {
      if(theRobotInfo.number == 1)
        goto PlaySupporter;
    }
    action
    {
      /**LookForward();/** The issue with staying in LookForward state is here **/
      Striker();
    }
  }

  state(PlayDefender)
  {
    action
    {
      LookForward();
      Defender();
    }
  }

  state(PlaySupporter)
  {
    transition
    {
      if(theRobotInfo.number == 1)
        goto PlayStriker;
    }
    action
    {
      LookForward();
      Supporter();
    }
  }
}
