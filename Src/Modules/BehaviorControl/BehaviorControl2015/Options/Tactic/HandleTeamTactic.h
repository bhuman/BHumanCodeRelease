option(HandleTeamTactic)
{
  initial_state(ChooseRole)
  {
    transition
    {
        goto PlaySupporter;
    }
    action
    {
      LookForward();
    }
  }
  
  state(PlayKeeper)
  {
    transition
    {
      if(action_aborted)
        if(LibTactic.nbOfKeeper >= 2)
          goto PlayStriker;
    }
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
      if(action_aborted)
        if(LibTactic.nbOfStriker == 0)
          goto PlayStriker;
        if(LibTactic.nbOfDefender >= 3)
          goto PlaySupporter;
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
      if(action_aborted)
        if(LibTactic.nbOfKeeper == 0)
          goto PlayKeeper;
        else if(LibTactic.nbOfStriker >= 2)
          goto PlayDefender;
        else if(!LibTactic.closerToTheBall)
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
      if(action_aborted)
        if(LibTactic.nbOfDefender <= 1)
          goto PlayDefender;
        else if(LibTactic.closerToTheBall)
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
