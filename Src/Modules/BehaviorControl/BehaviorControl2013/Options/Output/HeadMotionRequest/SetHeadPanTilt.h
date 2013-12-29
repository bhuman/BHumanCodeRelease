/** Sets all members of the HeadMotionRequest representation for positioning the robot's head. */
option(SetHeadPanTilt, float pan, float tilt, float speed)
{

  /** Set the head motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(state_time > 200 && !theHeadJointRequest.moving)
        goto targetReached;
    }
    action
    {
      theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
      theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
      theHeadMotionRequest.pan = pan;
      theHeadMotionRequest.tilt = tilt;
      theHeadMotionRequest.speed = speed;
    }
  }

  /** This state "notifies" the caller that
      the requested head angles are reached */
  target_state(targetReached)
  {
    transition
    {
      goto setRequest;
    }
    action
    {
      theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
      theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
      theHeadMotionRequest.pan = pan;
      theHeadMotionRequest.tilt = tilt;
      theHeadMotionRequest.speed = speed;
    }
  }
}
