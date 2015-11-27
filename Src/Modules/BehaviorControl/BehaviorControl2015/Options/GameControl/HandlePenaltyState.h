/** Handle penalty state (and the actions to be performed after leaving the penalty state).
*   This option is one level higher than the main game state handling as penalties
*   might occur in most game states. */
option(HandlePenaltyState)
{
  /** By default, the robot is not penalized and plays soccer according to the current game state.
     The chestbutton has to be pushed AND released to manually penalize a robot */
  initial_state(notPenalized)
  {
    transition
    {
      if(theRobotInfo.penalty != PENALTY_NONE)
        goto prePenalized;
    }
    action
    {
      HandleGameState();
    }
  }

  /** In case of any penalty, the robots stands still. Delay sound playback. */
  state(prePenalized)
  {
    transition
    {
      if(theRobotInfo.penalty == PENALTY_NONE)
        goto notPenalized;
      else if(state_time > 500)
        goto penalized;
    }
    action
    {
      SpecialAction(SpecialActionRequest::standHigh);
      theHeadControlMode = HeadControl::lookForward;
    }
  }

  /** In case of any penalty, the robots stands still. */
  state(penalized)
  {
    transition
    {
      if(theRobotInfo.penalty == PENALTY_NONE)
          goto unPenalized;
    }
    action
    {
      PlaySound("penalized.wav");
      SpecialAction(SpecialActionRequest::standHigh);
      theHeadControlMode = HeadControl::lookForward;
    }
  }

  /** If the penalty is removed say "not penalized" and continue. */
  state(unPenalized)
  {
    transition
    {     
      goto notPenalized;
    }
    action
    {
      PlaySound("penalized.wav");
      theHeadControlMode = HeadControl::lookForward;
      SpecialAction(SpecialActionRequest::standHigh);
    }
  }
}
