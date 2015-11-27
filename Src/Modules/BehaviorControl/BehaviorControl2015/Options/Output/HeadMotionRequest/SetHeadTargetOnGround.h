/** Sets all members of the HeadMotionRequest representation for positioning the robot's head pointing towards a given point on the field. */
option(SetHeadTargetOnGround, (const Vector3f&) target, ((HeadMotionRequest) CameraControlMode)(autoCamera) camera, (bool)(false) watchField, (float)(pi) speed)
{
  /** Set the head motion request. */
  initial_state(setRequest)
  {
    action
    {
      theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
      theHeadMotionRequest.cameraControlMode = camera;
      theHeadMotionRequest.watchField = watchField;
      theHeadMotionRequest.target = target;
      theHeadMotionRequest.speed = speed;
    }
  }
}
