/**
 * Kicks the Ball while moving towards the \c kickPose with the specified \c kickType.
 * @param kickType The WalkRequest::KickType to be executed
 * @param kickPose The Pose to move to
 */
option(InWalkKick, (const WalkKickVariant&) walkKick, (const Pose2f&) kickPose)
{
  /** Set the motion request / kickType. */
  initial_state(launch)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::walk && theMotionInfo.walkRequest.walkKickRequest == walkKick)
        goto execute;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = kickPose;
      theMotionRequest.walkRequest.speed = Pose2f(1.f, 1.f, 1.f);
      theMotionRequest.walkRequest.walkKickRequest = walkKick;
    }
  }

  /** Executes the kick */
  state(execute)
  {
    transition
    {
      if(theMotionInfo.walkRequest.walkKickRequest.kickType == WalkKicks::none)
        goto finished;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = kickPose;
      theMotionRequest.walkRequest.speed = Pose2f(1.f, 1.f, 1.f);
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
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
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    }
  }
}
