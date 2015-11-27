/** Sets all members of the MotionRequest representation for simple get up from engine*/
option(GetUpEngine)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::getUp)
        goto requestIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::getUp;
    }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion != MotionRequest::getUp)
        goto setRequest;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::getUp;
    }
  }
}
