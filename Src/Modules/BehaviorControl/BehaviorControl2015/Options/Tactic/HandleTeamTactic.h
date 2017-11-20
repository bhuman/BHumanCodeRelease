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
      LookForward();
      Striker();
    }
  }
}
