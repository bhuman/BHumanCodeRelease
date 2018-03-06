/** Fichier gerant les etapes dd'un Dive sur la Gauche (plonger, se relever) */
option(GroundPunchRight)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::groundPunchRight)
        goto requestRightIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::groundPunchRight;
    }
  }


  /** The motion process has started executing the request. */
  target_state(requestRightIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion =! MotionRequest::groundPunchRight)
        goto setRequest;
    }
    action
    {
      LookForward();
      Stand();;
    }
  }
}
