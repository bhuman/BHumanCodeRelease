/** Sets all members of the MotionRequest representation for simple sumoing */
option(Sumo)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::sumo)
        goto requestIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::sumo;
    }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion != MotionRequest::sumo)
        goto setRequest;
    }
    action
    {
      Stand();
      LookForward();
    }
  }
}
