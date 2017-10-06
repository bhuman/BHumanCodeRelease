/** Sets all members of the MotionRequest representation for simple standing */
option(Stand)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::stand)
        goto requestIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::stand;
    }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion != MotionRequest::stand)
        goto setRequest;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::stand;
    }
  }
}
