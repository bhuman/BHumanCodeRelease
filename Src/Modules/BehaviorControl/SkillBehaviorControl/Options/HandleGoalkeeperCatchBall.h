option(HandleGoalkeeperCatchBall)
{
  initial_state(notCatching)
  {
    transition
    {
      if(theGameState.isGoalkeeper() &&
         between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.3f, 3.f) &&
         theFieldBall.ballWasSeen(100) &&
         theFieldBall.isRollingTowardsOwnGoal &&
         theFieldBall.positionRelative.squaredNorm() < sqr(3000.f))
        goto preparingCatch;
    }
  }

  state(preparingCatch)
  {
    transition
    {
      if(!(between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.1f, 4.f) &&
           theFieldBall.ballWasSeen(100) &&
           theFieldBall.isRollingTowardsOwnGoal &&
           theFieldBall.positionRelative.squaredNorm() < sqr(3000.f)))
        goto notCatching;
      if(state_time >= 100)
        goto doingCatch;
    }
    action
    {
      theLookAtBallSkill();
      theKeyFrameArmsSkill({.motion = ArmKeyFrameRequest::keeperStand});
      theStandSkill();
    }
  }

  state(doingCatch)
  {
    transition
    {
      if(theInterceptBallSkill.isDone())
        goto notCatching;
    }
    action
    {
      unsigned interceptionMethods = bit(Interception::walk);
      if(theMotionInfo.executedPhase == MotionPhase::stand && theFrameInfo.getTimeSince(theMotionInfo.lastStandTimeStamp) > 1000.f)
        interceptionMethods |= bit(Interception::genuflectStandDefender);
      if(theLibPosition.isInOwnPenaltyArea(theRobotPose.translation))
      {
        if(!theGoaliePose.isNearLeftPost)
          interceptionMethods |= bit(Interception::jumpLeft);
        if(!theGoaliePose.isNearRightPost)
          interceptionMethods |= bit(Interception::jumpRight);
      }

      theInterceptBallSkill({.interceptionMethods = interceptionMethods,
                             .allowDive = theBehaviorParameters.keeperJumpingOn});
    }
  }
}
