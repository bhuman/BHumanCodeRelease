option(HandleCatchBall)
{
  initial_state(noCatch)
  {
    transition
    {
      if(between<float>(theFieldBall.intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance, theBehaviorParameters.ballCatchMaxWalkDistance) &&
         between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.5f, 3.0f) &&
         theFieldBall.ballWasSeen(100) &&
         theFallDownState.state == FallDownState::upright &&
         !theFieldBall.isRollingTowardsOpponentGoal &&
         theFieldBall.positionRelative.squaredNorm() < sqr(3000.f) &&
         theFieldBall.endPositionRelative.x() < -100.f)
        goto catching;
    }
  }

  state(catching)
  {
    transition
    {
      if(!between<float>(theFieldBall.intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance - 30.f, theBehaviorParameters.ballCatchMaxWalkDistance + 30.f) ||
         theFieldBall.isRollingTowardsOpponentGoal ||
         theFieldBall.endPositionRelative.x() > 0.f ||
         theInterceptBallSkill.isDone())
        goto noCatch;
    }
    action
    {
      unsigned interceptionMethods = bit(Interception::stand) | bit(Interception::walk);
      // Removed for 2021 competitions:
      // if(!theDamageConfigurationBody.noFieldGenuflect && theFrameInfo.getTimeSince(_context.behaviorStart) > genuflectDelay && theMotionInfo.executedPhase == MotionPhase::stand)
      //   interceptionMethods |= bit(Interception::genuflectStandDefender);
      theInterceptBallSkill(interceptionMethods);
    }
  }
}
