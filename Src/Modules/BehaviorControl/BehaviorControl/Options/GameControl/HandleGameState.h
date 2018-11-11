/** Triggers the options for the different game states.
 *  This option also invokes the get up behavior after a fall, as this is needed in most game states.
 */
option(HandleGameState)
{
  /** As game state changes are discrete external events and all states are independent of each other,
      a common transition can be used here. */
  common_transition
  {
    if(theGameInfo.state == STATE_INITIAL)
      goto initial;
    if(theGameInfo.state == STATE_FINISHED)
      goto finished;

    if(theFallDownState.state == FallDownState::fallen)
      goto getUp;

    if(theGameInfo.state == STATE_READY)
      goto ready;
    if(theGameInfo.state == STATE_SET)
      goto set;
    if(theGameInfo.state == STATE_PLAYING)
      goto playing;
  }

  /** Stand still and wait. */
  initial_state(initial)
  {
    action
    {
      HeadControlMode(HeadControl::none);
      SetHeadPanTilt(0.f, 0.f, 150_deg);
      SpecialAction(SpecialActionRequest::standHigh);
    }
  }

  /** Stand still and wait. If somebody wants to implement cheering moves => This is the place. ;-) */
  state(finished)
  {
    action
    {
      HeadControlMode(HeadControl::lookForward);
      Stand();
    }
  }

  /** Get up from the carpet. */
  state(getUp)
  {
    action
    {
      GetUp();
    }
  }

  /** Walk to kickoff position. */
  state(ready)
  {
    action
    {
      ReadyState();
    }
  }

  /** Stand and look around. */
  state(set)
  {
    action
    {
      Stand();
    }
  }

  /** Play soccer! */
  state(playing)
  {
    action
    {
      PlayingState();
    }
  }
}
