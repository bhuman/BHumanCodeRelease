option(HandleTeamTactic)
{
  initial_state(ChooseRole)
  {
    transition
    {
      if(theRobotInfo.number == 1)
        goto PlayKeeper;
      else if(theRobotInfo.number == 2)
        goto PlayStriker;
       else if(theRobotInfo.number == 3)
        goto PlayDefender;
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
}
