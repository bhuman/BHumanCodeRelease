/**
 * Kicks the Ball while moving towards the \c kickPose with the specified \c kickType.
 * @param kickType The WalkRequest::KickType to be executed
 * @param kickPose The Pose to move to
 */
option(InWalkKick, ((WalkRequest) KickType) kickType, (const Pose2f&) kickPose)
{
  /** Set the motion request / kickType. */
  initial_state(launch)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::walk && theMotionInfo.walkRequest.kickType == kickType)
        goto execute;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = kickPose;
      theMotionRequest.walkRequest.speed = Pose2f(1.f, 1.f, 1.f);
      theMotionRequest.walkRequest.kickType = kickType;
    }
  }

  /** Executes the kick */
  state(execute)
  {
    transition
    {
      if(theMotionInfo.walkRequest.kickType == WalkRequest::none)
        goto finished;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = kickPose;
      theMotionRequest.walkRequest.speed = Pose2f(1.f, 1.f, 1.f);
      theMotionRequest.walkRequest.kickType = WalkRequest::none;
    }
  }

  /** The kick has been executed */
  target_state(finished)
  {
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = kickPose;
      theMotionRequest.walkRequest.speed = Pose2f(1.f, 1.f, 1.f);
      theMotionRequest.walkRequest.kickType = WalkRequest::none;
    }
  }
}
