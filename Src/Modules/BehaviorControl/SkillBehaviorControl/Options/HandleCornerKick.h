option(HandleCornerKick)
{
  const Vector2f leftCorner = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline);
  const Vector2f rightCorner = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightSideline);
  const bool isLeftForward = theStrategyStatus.position == Tactic::Position::Type::forwardL;
  const bool isRightForward = theStrategyStatus.position == Tactic::Position::Type::forwardR;
  const float cornerPositionOffset = 500.f;
  const int refereeBallPlacementDelay = 5000;
  const float refereeBallPlacementAccuracy = 400.f;

  // teammates ball was in corner since set play started

  auto ballUnknownInCornerKick = [&](bool wasUnknown) -> bool
  {
    if(!(theGameState.isCornerKick() &&
         theGameState.isForOwnTeam() &&
         (isLeftForward || isRightForward)))
      return false;
    const Vector2f& corner = isLeftForward ? leftCorner : rightCorner;
    const float usedRefereeBallPlacementAccuracy = refereeBallPlacementAccuracy + (wasUnknown ? 0.f : 200.f);
    if(theTeammatesBallModel.isValid)
    {
      const bool teammatesBallModelIsInCorner = std::min((leftCorner - theTeammatesBallModel.position).squaredNorm(), (rightCorner - theTeammatesBallModel.position).squaredNorm()) < sqr(usedRefereeBallPlacementAccuracy);
      if(teammatesBallModelIsInCorner || theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) > refereeBallPlacementDelay)
        return false;
    }
    const bool ballModelIsInCorner = (corner - theFieldBall.positionOnField).squaredNorm() < sqr(usedRefereeBallPlacementAccuracy);
    return theBallModel.timeWhenLastSeen < theGameState.timeWhenStateStarted + (ballModelIsInCorner ? 0 : refereeBallPlacementDelay);
  };

  initial_state(ballValid)
  {
    transition
    {
      if(ballUnknownInCornerKick(false))
        goto searchCorner;
    }
  }

  state(searchCorner)
  {
    transition
    {
      if(!ballUnknownInCornerKick(true))
        goto ballValid;
    }

    action
    {
      const Vector2f cornerRelative = theRobotPose.inversePose * (isLeftForward ? leftCorner : rightCorner);
      const Vector2f targetRelative = theRobotPose.inversePose * ((isLeftForward ? leftCorner : rightCorner) - Vector2f(cornerPositionOffset, (isLeftForward ? cornerPositionOffset : -cornerPositionOffset)));
      theLookAtPointSkill({.target = (Vector3f() << cornerRelative, 0.f).finished()});
      theWalkToPointSkill({.target = {(cornerRelative).angle(), targetRelative},
                           .reduceWalkingSpeed = false });
    }
  }
}
