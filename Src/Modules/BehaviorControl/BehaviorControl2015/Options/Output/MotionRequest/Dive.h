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

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion != MotionRequest::dive)
        goto setRequest;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::dive;
    }
  }
}
