option(HandleTeamTactic)
{
  // Desactivated for testing
  //common_transition
  //{
  //if(LibInfo.oneGoaler)
  //      goto PlayKeeper;
  //}
    
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
      theBehaviorStatus.role = Role::keeper;
      LookForward();
      Keeper();
    }
  }
  
  state(PlayDefender)
  {
    transition
    {
      if(LibInfo.nbOfDef > 2)
        goto PlayStriker;
    }
    action
    {
      theBehaviorStatus.role = Role::defender;
      LookForward();
      Defender();
    }
  }
  
  state(PlayStriker)
  {
    transition
    {
      if(LibInfo.nbOfDef == 0)
        goto PlayDefender;
      else if(!LibInfo.closerToTheBall)
        goto PlaySupporter;
    }
    action
    {
      theBehaviorStatus.role = Role::striker;
      LookForward();
      Striker();
    }
  }

  state(PlaySupporter)
  {
    transition
    {
      if(LibInfo.nbOfDef <= 1)
        goto PlayDefender;
      else if(LibInfo.closerToTheBall)
        goto PlayStriker;
    }
    action
    {      
      theBehaviorStatus.role = Role::supporter;
      LookForward();
      Supporter();
    }
  }
}
