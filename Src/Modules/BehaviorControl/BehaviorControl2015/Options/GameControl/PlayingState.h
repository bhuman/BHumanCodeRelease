option(PlayingState)
{
  initial_state(demo)
  {
    transition
    {
      //if(!libDemo.parameters.isDemoActive)
      //  goto selectAction;
    }
    action
    {
      HandleTeamTactic();
    }
  }
}
