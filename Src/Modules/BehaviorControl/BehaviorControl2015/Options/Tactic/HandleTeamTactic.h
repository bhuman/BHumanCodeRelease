option(HandleTeamTactic)
{
  initial_state(ChooseRole)
  {
    transition
    {
      if(theRobotInfo.number == 1)
        goto PlayGoaler;
      else if(theRobotInfo.number == 2)
        goto PlayStriker;
    }
    action
    {
      LookForward();
    }
  }
  
  state(PlayGoaler)
  {
    action
    {
      LookForward();
      Goaler();
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
