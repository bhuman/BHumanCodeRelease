option(HandlePenaltyKick)
{
  initial_state(noPenaltyKick)
  {
    transition
    {
      if(theGameState.state == GameState::ownPenaltyKick && theSkillRequest.skill == SkillRequest::shoot)
        goto playingTaker;
      else if(theGameState.state == GameState::opponentPenaltyKick && theGameState.isGoalkeeper())
        goto playingKeeper;
    }
  }

  state(playingTaker)
  {
    transition
    {
      if(!theGameState.isPenaltyKick() && theFrameInfo.getTimeSince(theExtendedGameState.timeWhenStateStarted[GameState::playing]) > 1000.f)
        goto noPenaltyKick;
    }
    action
    {
      PenaltyTaker();
    }
  }

  state(playingKeeper)
  {
    transition
    {
      if(!theGameState.isPenaltyKick() && (theFrameInfo.getTimeSince(theExtendedGameState.timeWhenStateStarted[GameState::playing]) > 1000.f || theInterceptBallSkill.isDone()))
        goto noPenaltyKick;
    }
    action
    {
      PenaltyKeeper();
    }
  }
}
