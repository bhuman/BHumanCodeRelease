/** Sets all members of the MotionRequest representation for simple get up from engine*/
option(GetUpEngine)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theGetUpEngineOutput.isLeavingPossible == false)
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
      if(theGetUpEngineOutput.isLeavingPossible == true)
        goto requestIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::getUp;
    }
  }

  target_state(requestIsExecuted)
  {
    action
    {
      theMotionRequest.motion = MotionRequest::getUp;
    }
  }
}
