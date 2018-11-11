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
        goto penalized;
    }
    action
    {
      HandleGameState();
    }
  }

  /** In case of any penalty, the robots stands still. */
  state(penalized)
  {
    transition
    {
      if(theRobotInfo.penalty == PENALTY_NONE)
        goto preUnpenalize;
    }
    action
    {
      PlaySound("penalized.wav");
      SpecialAction(SpecialActionRequest::standHigh);
      HeadControlMode(HeadControl::lookForward);
    }
  }

  state(preUnpenalize)
  {
    transition
    {
      if(theRobotInfo.penalty != PENALTY_NONE)
        goto penalized;
      else if(theGameInfo.state == STATE_INITIAL || state_time > 500)
        goto notPenalized;
    }
    action
    {
      PlaySound("notPenalized.wav");
      HeadControlMode(HeadControl::lookForward);
      Stand();
    }
  }
}
