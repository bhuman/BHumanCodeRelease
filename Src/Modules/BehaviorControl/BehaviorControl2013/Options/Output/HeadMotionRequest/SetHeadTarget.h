/** Sets all members of the HeadMotionRequest representation for positioning the robot's head pointing towards a given point relative to the robot's hip. */
option(SetHeadTarget, const Vector3<>& target, HeadMotionRequest::CameraControlMode camera = HeadMotionRequest::autoCamera, bool watchField = false, float speed = pi)
{

  /** Set the head motion request. */
  initial_state(setRequest)
  {
    action
    {
      theHeadMotionRequest.mode = HeadMotionRequest::targetMode;
      theHeadMotionRequest.cameraControlMode = camera;
      theHeadMotionRequest.watchField = watchField;
      theHeadMotionRequest.target = target;
      theHeadMotionRequest.speed = speed;
    }
  }

}
