option(HandleReplayWalk)
{
  initial_state(inactive)
  {
    transition
    {
      goto replay;
    }
  }

  state(replay)
  {
    action
    {
      theReplayWalkSkill();
      theLookForwardSkill();
    }
  }
}
