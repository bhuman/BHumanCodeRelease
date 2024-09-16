#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandlePenaltyKick,
       defs((int)(1000) takerReturnDelay, /**< The penalty taker will stay in this option for this duration even after the switch to Playing. */
            (int)(1000) keeperReturnDelay)) /**< The penalty keeper will stay in this option for this duration even after the switch to Playing. */
{
  initial_state(noPenaltyKick)
  {
    transition
    {
      if(theGameState.state == GameState::ownPenaltyKick &&
         (theSkillRequest.skill == SkillRequest::shoot ||
          theSkillRequest.skill == SkillRequest::pass ||
          theSkillRequest.skill == SkillRequest::dribble ||
          theSkillRequest.skill == SkillRequest::clear))
        goto playingTaker;
      else if(theGameState.state == GameState::opponentPenaltyKick && theGameState.isGoalkeeper())
        goto playingKeeper;
    }
  }

  state(playingTaker)
  {
    transition
    {
      if(theGameState.state != GameState::ownPenaltyKick &&
         (theGameState.state != GameState::playing ||
          theFrameInfo.getTimeSince(theExtendedGameState.timeWhenStateStarted[GameState::playing]) > takerReturnDelay))
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
      if(theGameState.state != GameState::opponentPenaltyKick &&
         (theGameState.state != GameState::playing ||
          theFrameInfo.getTimeSince(theExtendedGameState.timeWhenStateStarted[GameState::playing]) > keeperReturnDelay ||
          action_done))
        goto noPenaltyKick;
    }
    action
    {
      PenaltyKeeper();
    }
  }
}
