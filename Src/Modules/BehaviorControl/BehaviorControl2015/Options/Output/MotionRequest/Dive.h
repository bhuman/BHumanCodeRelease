/** Sets all members of the MotionRequest representation for simple standing */
option(Dive)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::dive)
        goto requestIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::dive;
    }
  }

  /** Get up */
  state(standUp)
  {
    transition
    {
      if(action_done)
        goto stand;
    }
    action
    {
      LookForward();
      GetUpEngine();
    }
  }

   /** Try to stand
   *  If the robot still lies down again, try to
   *  stand up again (this changes only anything if the robot is still on the floor after pick up (and nobody sent penalize and game-state is still playing)*/
  
 state(stand)
  {
    transition
    {
      if(theFallDownState.state == FallDownState::onGround)
        goto standUp;
    }
    action
    {
      LookForward();
      Stand();
    }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted)
  {
    transition
    {
      if(theFallDownState.state == FallDownState::onGround)
        goto standUp;
    }
    action
    {
      goto requestIsExecuted
    }
  }
}
