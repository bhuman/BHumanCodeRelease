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
      if(libInfo.nbOfDef > 1)
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
      if(libInfo.nbOfDef == 0)
        goto PlayDefender;
      else if(!libInfo.closerToTheBall)
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
      if(libInfo.nbOfDef <= 1)
        goto PlayDefender;
      else if(libInfo.closerToTheBall)
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
