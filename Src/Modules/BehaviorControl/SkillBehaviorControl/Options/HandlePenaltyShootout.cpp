#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandlePenaltyShootout)
{
  common_transition
  {
    if(!theGameState.isPenaltyShootout())
      goto noPenaltyShootout;
    else if(theGameState.isInitial())
      goto initial;
    else if(theGameState.isReady() || theGameState.isSet())
      goto set;
    else if(theGameState.isPlaying())
    {
      if(theGameState.isForOwnTeam())
        goto playingTaker;
      else
        goto playingKeeper;
    }
    else if(theGameState.isFinished())
      goto finished;
    FAIL("Unknown game state.");
  }

  initial_state(noPenaltyShootout)
  {}

  state(initial)
  {
    action
    {
      LookAtAngles({.pan = 0_deg,
                    .tilt = 0_deg,
                    .speed = 150_deg});
      Stand({.high = true});
    }
  }

  state(set)
  {
    action
    {
      LookForward();
      Stand({.high = true});
    }
  }

  state(playingTaker)
  {
    action
    {
      PenaltyTaker();
    }
  }

  state(playingKeeper)
  {
    action
    {
      PenaltyKeeper();
    }
  }

  state(finished)
  {
    action
    {
      LookForward();
      Stand({.high = true});
    }
  }
}
