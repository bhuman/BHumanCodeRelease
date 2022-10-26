option(PenaltyKeeper)
{
  // Martin Kroker, 14.04.2013?
  auto penaltyKeeperShouldCatchBall = [&]
  {
    float offsetToYAxis = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGoalArea, 0.f)).x();
    if(offsetToYAxis < 0.f)
      offsetToYAxis = 0.f;

    float timeToIntersectYAxis = std::numeric_limits<float>::max();
    const Vector2f& ballPositionRel = theBallModel.estimate.position;
    const Vector2f& ballVelocityRel = theBallModel.estimate.velocity;
    // Return false if the ball does not move or moves away from the robot.
    if(ballVelocityRel.x() >= 0.f)
      return false;
    // Return false if the ball has already intersected the line.
    if((ballPositionRel.x() < 0.f && offsetToYAxis < ballPositionRel.x()) ||
       (ballPositionRel.x() > 0.f && offsetToYAxis > ballPositionRel.x()))
      return false;

    Vector2f ballToIntersectPositionVector = theFieldBall.intersectionPositionWithOwnYAxis - ballPositionRel;
    const float vectorLengthFactor = (ballPositionRel.x() - offsetToYAxis) / ballToIntersectPositionVector.x();
    ballToIntersectPositionVector *= vectorLengthFactor;
    const float s = ballToIntersectPositionVector.norm();
    const float a = theBallSpecification.friction;
    ASSERT(a < 0.f);
    timeToIntersectYAxis = BallPhysics::timeForDistance(ballVelocityRel, s, a);

    return between<float>(timeToIntersectYAxis, 0.01f, 10.5f) && theFieldBall.ballWasSeen(500) && (theGameState.isPenaltyShootout() || theFieldBall.isRollingTowardsOwnGoal);
  };

  initial_state(initial)
  {
    transition
    {
      if(penaltyKeeperShouldCatchBall())
        goto intercept;
    }
    action
    {
      theLookForwardSkill();
      if(theGameState.isPenaltyShootout())
        theDiveSkill({ .request = MotionRequest::Dive::prepare });
      else
        theStandSkill();
    }
  }

  state(intercept)
  {
    action
    {
      unsigned interceptionMethods = bit(Interception::jumpLeft) | bit(Interception::jumpRight);
      if(!theGameState.isPenaltyShootout())
        interceptionMethods |= bit(Interception::genuflectStand) | bit(Interception::walk);
      theInterceptBallSkill({.interceptionMethods = interceptionMethods,
                             .allowGetUp = !theGameState.isPenaltyShootout(),
                             .allowDive = theBehaviorParameters.keeperJumpingOn});
    }
  }
}
