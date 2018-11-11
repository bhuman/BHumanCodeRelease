/** Sets all members of the MotionRequest representation for simple get up from engine*/
option(GetUpEngine)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionRequest.motion == MotionRequest::getUp && theMotionInfo.motion == MotionRequest::getUp)
        goto executing;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::getUp;
    }
  }

  /** The motion process has started executing the request. */
  state(executing)
  {
    transition
    {
      if(theMotionRequest.motion != MotionRequest::getUp || theMotionInfo.motion != MotionRequest::getUp)
        goto setRequest;

      if(theGetUpEngineOutput.isLeavingPossible == true)
        goto upright;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::getUp;
    }
  }

  target_state(upright)
  {
    transition
    {
      if(state_time > 2000 && theFallDownState.state == FallDownState::fallen)
        goto setRequest;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::stand;
    }
  }

}
