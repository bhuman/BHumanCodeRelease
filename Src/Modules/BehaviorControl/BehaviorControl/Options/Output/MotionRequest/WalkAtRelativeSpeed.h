/** Sets all members of the MotionRequest representation for executing a relativeSpeedMode-WalkRequest
 *  (i.e. Walk at a \c speed)
 *  @param speed Walking speeds, in the range [-1.f .. 1.f].
 *                e.g.  Pose2f(0.f, 1.f, 0.f) lets move the robot forward at full speed
 *                      Pose2f(0.f, 0.5f, 0.5f) lets move the robot diagonal at half of the possible speed
 *                      Pose2f(0.5f, 1.f, 0.f) lets move the robot in a circle
 */
option(WalkAtRelativeSpeed, (const Pose2f&) speed)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::walk)
        goto requestIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::relativeSpeedMode;
      theMotionRequest.walkRequest.speed = speed;
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion != MotionRequest::walk)
        goto setRequest;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::relativeSpeedMode;
      theMotionRequest.walkRequest.speed = speed;
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    }
  }
}
