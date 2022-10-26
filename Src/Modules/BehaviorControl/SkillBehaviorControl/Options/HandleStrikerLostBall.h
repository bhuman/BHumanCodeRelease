option(HandleStrikerLostBall)
{
  enum TurnDirection
  {
    none, /**< The robot should not turn to recover the ball because it is not known (yet) on which side it probably is. */
    left, /**< The robot should turn left. */
    right, /**< The robot should turn right. */
  };

  float maxObstacleToBallDistance = 500.f; /**< This option can only become active if there is an obstacle within this distance from the ball. */
  int minBallDisappearedTime = 500; /**< This option can become active after the ball has disappeared for this time. */
  float maxDistanceToBall = 700.f; /**< This option can only become active if the ball is closer than this. */
  float minEccentricityToAssumeSide = 0.15f; /**< If the ball is further away from the image center than this fraction of the image width, its side is used to determine the turn direction. */
  Angle minAngleToAssumeSide = 20_deg; /**< If the ball has a larger angle than this to the x axis of the robot in the moment of losing it, its side is used to determine the turn direction. */

  const bool playBall = theSkillRequest.skill == SkillRequest::shoot || theSkillRequest.skill == SkillRequest::pass || theSkillRequest.skill == SkillRequest::dribble;
  thread_local static TurnDirection lastTurnDirection;

  if(theBallModel.timeWhenLastSeen == theFrameInfo.time)
  {
    Vector2f ballInImage;
    const float imageCenterX = static_cast<float>(theCameraInfo.width / 2);
    const float imageEccentricityX = theCameraInfo.width * minEccentricityToAssumeSide;
    if(Transformation::robotToImage(Vector3f(theBallModel.lastPerception.x(), theBallModel.lastPerception.y(), theBallSpecification.radius), theCameraMatrix, theCameraInfo, ballInImage) &&
       (ballInImage.x() < imageCenterX - imageEccentricityX || ballInImage.x() > imageCenterX + imageEccentricityX))
      lastTurnDirection = ballInImage.x() < imageCenterX ? left : right;
    else if(std::abs(theFieldBall.positionRelative.angle()) > minAngleToAssumeSide)
      lastTurnDirection = theFieldBall.positionRelative.y() > 0.f ? left : right;
    else
      lastTurnDirection = none;
  }

  initial_state(knownBallPosition)
  {
    transition
    {
      auto thereIsAnObstacleNearTheBall = [&]() -> bool
      {
        for(const Obstacle& o : theObstacleModel.obstacles)
          if((o.center - theFieldBall.positionRelative).squaredNorm() < sqr(maxObstacleToBallDistance))
            return true;
        return false;
      };

      auto ballIsOccludedByAnObstacle = [this]() -> bool
      {
        const Angle ballAngle = theFieldBall.positionRelative.angle();
        const float ballDistance = theFieldBall.positionRelative.norm();
        for(const Obstacle& o : theObstacleModel.obstacles)
          if(Rangea(o.right.angle(), o.left.angle()).isInside(ballAngle) && o.center.norm() < ballDistance - 100.f)
            return true;
        return false;
      };

      if(playBall &&
         theFieldBall.timeSinceBallDisappeared > minBallDisappearedTime &&
         //theExtendedGameInfo.timeWhenLastSearchedBall < theBallModel.timeWhenLastSeen &&
         theFieldBall.positionRelative.squaredNorm() < sqr(maxDistanceToBall) &&
         thereIsAnObstacleNearTheBall() &&
         !ballIsOccludedByAnObstacle())
      {
        goto searchForBall;
      }
    }
  }

  state(searchForBall)
  {
    transition
    {
      if(!playBall ||
         theFieldBall.ballWasSeen() ||
         state_time > 3000)
      {
        goto knownBallPosition;
      }
    }

    action
    {
      const TurnDirection turnDirection = lastTurnDirection;
      const float angleFactor = turnDirection == none ? 0.f : (turnDirection == left ? 1.f : -1.f);
      // theActivitySkill(BehaviorStatus::nearFieldSearchForBall);
      if(turnDirection == none)
        theLookLeftAndRightSkill(theFieldBall.positionRelative.y() > 0.f); // Even if nothing is known, this might be better than nothing.
      else
        theLookAtAnglesSkill(turnDirection == left ? 45_deg : -45_deg, 0.38f, 180_deg);
      theWalkToPointSkill({ .target = Pose2f(angleFactor * 90_deg, -300.f, 0.f),
                            .speed = 0.7f,
                            .reduceWalkingSpeed = false,
                            .rough = false,
                            .disableObstacleAvoidance = false,
                            .disableAligning = true });
    }
  }
}
