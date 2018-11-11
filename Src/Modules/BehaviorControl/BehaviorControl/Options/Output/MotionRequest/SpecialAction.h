/** Sets all members of the MotionRequest representation for executing a SpecialAction */
option(SpecialAction, (SpecialActionRequest::SpecialActionID) id, (bool)(false) mirror)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::specialAction &&
         theMotionInfo.specialActionRequest.specialAction == id &&
         theMotionInfo.specialActionRequest.mirror == mirror)
        goto requestIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::specialAction;
      theMotionRequest.specialActionRequest.specialAction = id;
      theMotionRequest.specialActionRequest.mirror = mirror;
    }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion != MotionRequest::specialAction ||
         theMotionInfo.specialActionRequest.specialAction != id ||
         theMotionInfo.specialActionRequest.mirror != mirror)
        goto setRequest;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::specialAction;
      theMotionRequest.specialActionRequest.specialAction = id;
      theMotionRequest.specialActionRequest.mirror = mirror;
    }
  }
}
