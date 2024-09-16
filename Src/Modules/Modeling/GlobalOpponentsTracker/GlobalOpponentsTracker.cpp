/**
 * @file GlobalOpponentsTracker.cpp
 *
 * Implementation of a module that aims to track all opponent robots on the pitch
 *
 * @author Florian Maaß
 * @author Jan Fiedler & Nicole Schrader
 * @author Tim Laue
 * @author Michelle Gusev
 */

#include "GlobalOpponentsTracker.h"
#include "Debugging/DebugDrawings.h"

MAKE_MODULE(GlobalOpponentsTracker);

GlobalOpponentsTracker::GlobalOpponentsTracker()
{
  // Set the zones in which robots are situated during a penalty
  // and when returning from a penalty.
  // The exact positions depend on the referees, thus the hardcoded values
  // here are rough guesses that should cover most situations.
  const Geometry::Rect penaltyPlacementLeftOpp(Vector2f(1000.f, theFieldDimensions.yPosLeftTouchline + 200.f),
    Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder + 100.f));
  const Geometry::Rect penaltyPlacementRightOpp(Vector2f(1000.f, theFieldDimensions.yPosRightTouchline - 200.f),
    Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightFieldBorder - 100.f));
  const Geometry::Rect returnFromPenaltyLeftOpp(Vector2f(theFieldDimensions.xPosOpponentPenaltyMark - 700.f, theFieldDimensions.yPosLeftTouchline - 200),
    Vector2f(theFieldDimensions.xPosOpponentPenaltyMark + 700.f, theFieldDimensions.yPosLeftTouchline + 400.f));
  const Geometry::Rect returnFromPenaltyRightOpp(Vector2f(theFieldDimensions.xPosOpponentPenaltyMark - 700.f, theFieldDimensions.yPosRightTouchline - 400.f),
    Vector2f(theFieldDimensions.xPosOpponentPenaltyMark + 700.f, theFieldDimensions.yPosRightTouchline + 200));
  penalizedRobotZonesOpponentTeam.push_back(penaltyPlacementLeftOpp);
  penalizedRobotZonesOpponentTeam.push_back(penaltyPlacementRightOpp);
  returnFromPenaltyZonesOpponentTeam.push_back(returnFromPenaltyLeftOpp);
  returnFromPenaltyZonesOpponentTeam.push_back(returnFromPenaltyRightOpp);
}

void GlobalOpponentsTracker::update(GlobalOpponentsModel& globalOpponentsModel)
{
  DECLARE_DEBUG_DRAWING("module:GlobalOpponentsTracker:penaltyZones", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalOpponentsTracker:internalOpponents", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalOpponentsTracker:pooledOpponents", "drawingOnField");

  if(clearAndFinish(globalOpponentsModel))
    return;

  deleteObstacles(); // Delete old obstacles and obstacles that are no longer hypotheses.
  dynamic(); // Apply extended kalman filter prediction step to hypotheses.
  if(useArmContactModel)
    addArmContacts(); // Add hypotheses measured by arm contact.
  if(useFootBumperState)
    addFootContacts(); // Add hypotheses measured by foot contact.
  addPlayerPercepts(); // Add players field percepts.
  mergeOverlapping(); // Overlapping hypotheses are merged together.

  // Set the same length for left and right.
  for(auto& obstacle : obstacleHypotheses)
    obstacle.setLeftRight((obstacle.left - obstacle.right).norm() * .5f);

  shouldBeSeen(); // Mark obstacles that should be seen but weren't seen recently.

  updateGameInfo();
  fillModel(globalOpponentsModel);
  draw();
}

void GlobalOpponentsTracker::fillModel(GlobalOpponentsModel& globalOpponentsModel)
{
  // Update obstacles from valid hypotheses:
  globalOpponentsModel.opponents.clear();
  for(const auto& ob : obstacleHypotheses)
  {
    if(!ob.isTeammate() && ob.seenCount >= minPercepts && !shouldIgnore(ob))
    {
      GlobalOpponentsModel::OpponentEstimate opponent;
      opponent.position = theRobotPose * ob.center;
      opponent.left = theRobotPose * ob.left;
      opponent.right = theRobotPose * ob.right;
      opponent.unidentified = ob.isUnknown() || ob.isSomeRobot();
      globalOpponentsModel.opponents.emplace_back(opponent);
    }
  }
  // Set fields about opponent numbers:
  globalOpponentsModel.numOfUnknownOpponents = numberOfUnpenalizedOpponents;
  globalOpponentsModel.numOfPenalizedOpponents = numberOfPenalizedOpponents;
}

void GlobalOpponentsTracker::updateGameInfo()
{
  // Opponents: **********
  int totalNumberOfOpponents = 0;
  numberOfUnpenalizedOpponents = 0;
  // Loop through list of opponents and count all those that are not substitutes
  // as well those that are currently supposed to play:
  for(int i = 0; i < MAX_NUM_PLAYERS; i++)
  {
    if(theGameState.opponentTeam.playerStates[i] != GameState::substitute)
    {
      totalNumberOfOpponents++;
      if(!GameState::isPenalized(theGameState.opponentTeam.playerStates[i]) ||                    // Playing
         theGameState.opponentTeam.playerStates[i] == GameState::penalizedIllegalMotionInSet ||   // Not playing but still on pitch
         theGameState.opponentTeam.playerStates[i] == GameState::penalizedIllegalMotionInStandby)
        numberOfUnpenalizedOpponents++;
    }
  }
  numberOfPenalizedOpponents = totalNumberOfOpponents - numberOfUnpenalizedOpponents;
}

bool GlobalOpponentsTracker::clearAndFinish(GlobalOpponentsModel& globalOpponentsModel)
{
  DEBUG_RESPONSE_ONCE("module:ObstacleModelProvider:clear")
    obstacleHypotheses.clear();

  if(theGameState.isPenalized()
    || theGameState.isInitial()
    // While falling down / getting up / the obstacles might be invalid, better clean up
    || theFallDownState.state == FallDownState::falling
    || theFallDownState.state == FallDownState::fallen
    || theMotionInfo.executedPhase == MotionPhase::getUp
    || theGameState.isPenaltyShootout()) // Penalty shootout -> obstacles will be ignored
  {
    if(!theGameState.isFinished()) // If the GameController operator fails epically and resets from finished to playing
    {
      globalOpponentsModel.opponents.clear();
      obstacleHypotheses.clear();
    }
    return true;
  }
  return false;
}

void GlobalOpponentsTracker::deleteObstacles()
{
  for(auto obstacle = obstacleHypotheses.begin(); obstacle != obstacleHypotheses.end();)
  {
    const float obstacleRadius = Obstacle::getRobotDepth();
    const float centerDistanceSquared = obstacle->center.squaredNorm();
    Vector2f absObsPos = theRobotPose * obstacle->center;
    if(obstacle->notSeenButShouldSeenCount >= notSeenThreshold
      || theFrameInfo.getTimeSince(obstacle->lastSeen) >= deleteAfter
      || centerDistanceSquared >= sqr(maxOpponentDistance)
      || centerDistanceSquared <= sqr(obstacleRadius * 0.5f) // Obstacle is really inside us
      // HACK: Ignore the referee hand before the kick-off.
      || (!theExtendedGameState.wasPlaying() && theGameState.isPlaying() && theFrameInfo.getTimeSince(obstacle->lastSeen) > 1500)
      || theFieldDimensions.clipToField(absObsPos) > 500.f // obstacleIsNotOnField
      )
    {
      obstacle = obstacleHypotheses.erase(obstacle);
    }
    else
      ++obstacle;
  }
}

void GlobalOpponentsTracker::dynamic()
{
  // Obstacle has to move in the opposite direction.
  const float odometryRotation = -theOdometer.odometryOffset.rotation;
  const Vector2f odometryTranslation = -theOdometer.odometryOffset.translation.rotated(odometryRotation);
  Matrix2f odometryJacobian;
  odometryJacobian << std::cos(odometryRotation), -std::sin(odometryRotation), std::sin(odometryRotation), std::cos(odometryRotation);
  // Noise
  // TODO Add noise from rotation
  const float odometryDeviationX = sqr(odometryTranslation.x() * odoDeviation.x());
  const float odometryDeviationY = sqr(odometryTranslation.y() * odoDeviation.y());
  // Process noise
  const float odometryNoiseX = odometryDeviationX + sqr(pNp);
  const float odometryNoiseY = odometryDeviationY + sqr(pNp);

  for(GlobalOpponentsHypothesis& obstacle : obstacleHypotheses)
    obstacle.dynamic(odometryRotation, odometryTranslation, odometryJacobian, odometryNoiseX, odometryNoiseY);
}

void GlobalOpponentsTracker::addArmContacts()
{
  merged.clear();
  merged.resize(obstacleHypotheses.size(), false);

  FOREACH_ENUM(Arms::Arm, arm)
  {
    if(theArmContactModel.status[arm].contact && theFrameInfo.getTimeSince(theArmContactModel.status[arm].timeOfLastContact) <= maxContactTime)
    {
      if(!armContact[arm])
      {
        armContact[arm] = true;
      }
      Vector2f center = (theTorsoMatrix.inverse() * theRobotModel.limbs[Limbs::combine(arm, Limbs::shoulder)].translation).topRows(2);
      center.y() += sgn(center.y()) * (Obstacle::getRobotDepth() + 15.f);
      GlobalOpponentsHypothesis obstacle(armCov, center, Vector2f::Zero(), Vector2f::Zero(), theFrameInfo.time, Obstacle::unknown, 1);
      obstacle.setLeftRight(Obstacle::getRobotDepth());
      // Insert valid obstacle.
      tryToMerge(obstacle);
    }
    else
      armContact[arm] = false;
  };
}

void GlobalOpponentsTracker::addFootContacts()
{
  merged.clear();
  merged.resize(obstacleHypotheses.size(), false);

  FOREACH_ENUM(Legs::Leg, leg)
  {
    if(theFootBumperState.status[leg].contact && theFrameInfo.getTimeSince(theFootBumperState.status[leg].lastContact) <= maxContactTime)
    {
      if(!footContact[leg])
      {
        footContact[leg] = true;
      }
      Vector2f center = (theTorsoMatrix.inverse() * theRobotModel.limbs[Limbs::combine(leg, Limbs::foot)].translation).topRows(2);
      center.x() += Obstacle::getRobotDepth() + distJointToToe + distToeToBumper;
      GlobalOpponentsHypothesis obstacle(feetCov, center, Vector2f::Zero(), Vector2f::Zero(), theFrameInfo.time, Obstacle::unknown, 1);
      obstacle.setLeftRight(Obstacle::getRobotDepth());
      // Insert valid obstacle.
      tryToMerge(obstacle);
    }
    else
      footContact[leg] = false;
  };
}

void GlobalOpponentsTracker::addPlayerPercepts()
{
  if(theObstaclesFieldPercept.obstacles.empty())
    return;

  merged.clear();
  merged.resize(obstacleHypotheses.size(), false);

  for(const ObstaclesFieldPercept::Obstacle& percept : theObstaclesFieldPercept.obstacles)
  {
    // Too far away?
    if(percept.center.squaredNorm() >= sqr(maxOpponentDistance))
      continue;

    // TODO: Consider handling the goalkeepers in a different way.
    Obstacle::Type type = (percept.type == ObstaclesFieldPercept::opponentPlayer || percept.type == ObstaclesFieldPercept::opponentGoalkeeper) ? (percept.fallen ? Obstacle::fallenOpponent : Obstacle::opponent)
      : ((percept.type == ObstaclesFieldPercept::ownPlayer || percept.type == ObstaclesFieldPercept::ownGoalkeeper) ? (percept.fallen ? Obstacle::fallenTeammate : Obstacle::teammate)
        : (percept.fallen ? Obstacle::fallenSomeRobot : Obstacle::someRobot));

    GlobalOpponentsHypothesis obstacle(percept.covariance, percept.center,
      percept.left.normalized(percept.left.norm() + Obstacle::getRobotDepth()),
      percept.right.normalized(percept.right.norm() + Obstacle::getRobotDepth()), theFrameInfo.time, type, 1);

    // Obstacles have a minimum size
    if((obstacle.left - obstacle.right).squaredNorm() < sqr(2 * Obstacle::getRobotDepth()))
      obstacle.setLeftRight(Obstacle::getRobotDepth());
    tryToMerge(obstacle);
  }
}

void GlobalOpponentsTracker::tryToMerge(const GlobalOpponentsHypothesis& measurement)
{
  if(obstacleHypotheses.empty())
  {
    obstacleHypotheses.emplace_back(measurement);
    merged.push_back(true);
    return;
  }

  const float mergeDistanceSquared = sqr(calculateMergeRadius(measurement.center));
  float possibleMergeDistSquared = std::numeric_limits<float>::max();
  std::size_t atMerge = 0; // Element matching the merge condition
  // Search for the possible obstacle for merging.
  for(std::size_t i = 0; i < obstacleHypotheses.size(); ++i)
  {
    if(merged[i])
      continue;

    const float distanceSquared = (measurement.center - obstacleHypotheses[i].center).squaredNorm();

    // Found probably matching obstacle.
    if(distanceSquared <= mergeDistanceSquared && distanceSquared <= possibleMergeDistSquared)
    {
      possibleMergeDistSquared = distanceSquared;
      atMerge = i;
    }
  }

  // Merge
  if(possibleMergeDistSquared < std::numeric_limits<float>::max())
  {
    LINE("module:ObstacleModelProvider:merge", measurement.center.x(), measurement.center.y(),
      obstacleHypotheses[atMerge].center.x(), obstacleHypotheses[atMerge].center.y(), 10, Drawings::dashedPen, ColorRGBA::red);

    obstacleHypotheses[atMerge].lastSeen = measurement.lastSeen;

    obstacleHypotheses[atMerge].measurement(measurement, modelWidthWeighting); // EKF
    obstacleHypotheses[atMerge].determineAndSetType(measurement, teamThreshold, uprightThreshold);
    obstacleHypotheses[atMerge].seenCount += measurement.seenCount;
    obstacleHypotheses[atMerge].notSeenButShouldSeenCount = 0; // Reset that counter.
    merged[atMerge] = true;
    return;
  }

  // Did not find possible match.
  obstacleHypotheses.emplace_back(measurement);
  merged.push_back(true);
}

void GlobalOpponentsTracker::mergeOverlapping()
{
  // TODO The merge with velocity is still missing here. The reason for this is that tests are necessary to see how precise it is.
  // If this is deemed to be sufficient, rows of obstacles with robots running in parallel can be prevented.
  // In addition, the maximum velocity, which is valid, is not yet handled.
  if(obstacleHypotheses.size() < 2)
    return;

  for(std::size_t i = 0; i < obstacleHypotheses.size(); ++i)
  {
    GlobalOpponentsHypothesis& actual = obstacleHypotheses[i];
    for(std::size_t j = obstacleHypotheses.size() - 1; j > i; --j)
    {
      const GlobalOpponentsHypothesis& other = obstacleHypotheses[j];

      // Continue with the next obstacles if they were last seen almost at the same time, as there are probably really two of them.
      if(std::max(actual.lastSeen, other.lastSeen) - std::min(actual.lastSeen, other.lastSeen) < mergeOverlapTimeDiff)
        continue;

      // The sum of the radius of the obstacles.
      const float overlap = ((actual.left - actual.right).norm() + (other.left - other.right).norm()) * .5f;
      // The distance of the centers
      const float distanceOfCenters = (other.center - actual.center).norm();

      // Merge the obstacles.
      if(((distanceOfCenters <= overlap || distanceOfCenters < 2 * Obstacle::getRobotDepth()) // The obstacles are overlapping
        || (actual.squaredMahalanobis(other) < sqr(minMahalanobisDistance)
          && (actual.seenCount >= minPercepts && other.seenCount >= minPercepts))) // they were seen at least minPercepts times
        && (actual.isUnknown() || actual.isSomeRobot() || other.isUnknown() || other.isSomeRobot()
          || actual.type == other.type)) // Their type is unknown, someRobot or fallenSomeRobot or their type is equal
      {
        Obstacle::fusion2D(actual, other);
        // Since fusion2D makes all previous positions unusable for a correct calculation.
        actual.determineAndSetType(other, teamThreshold, uprightThreshold);
        actual.lastSeen = std::max(actual.lastSeen, other.lastSeen);
        actual.seenCount = std::max(actual.seenCount, other.seenCount);
        actual.notSeenButShouldSeenCount = (actual.notSeenButShouldSeenCount + other.notSeenButShouldSeenCount) / 2;
        obstacleHypotheses.erase(obstacleHypotheses.begin() + j);
      }
    }
  }
}

void GlobalOpponentsTracker::shouldBeSeen()
{
  if(obstacleHypotheses.empty())
    return;

  const float cameraAngle = theCameraMatrix.rotation.getZAngle();
  const float cameraAngleLeft = cameraAngle + theCameraInfo.openingAngleWidth * cameraAngleFactor,
    cameraAngleRight = cameraAngle - theCameraInfo.openingAngleWidth * cameraAngleFactor;

  COMPLEX_DRAWING("module:ObstacleModelProvider:cameraAngle")
  {
    // The camera angle being used
    Vector2f camLeft(maxOpponentDistance, 0.f);
    Vector2f camRight(maxOpponentDistance, 0.f);
    camLeft = camLeft.rotate(cameraAngleLeft);
    camRight = camRight.rotate(cameraAngleRight);
    const ColorRGBA cameraColor = theCameraInfo.camera == CameraInfo::upper ? ColorRGBA::blue : ColorRGBA::yellow;
    LINE("module:ObstacleModelProvider:cameraAngle", 0, 0, camLeft.x(), camLeft.y(), 10, Drawings::solidPen, cameraColor);
    LINE("module:ObstacleModelProvider:cameraAngle", 0, 0, camRight.x(), camRight.y(), 10, Drawings::solidPen, cameraColor);
  }

  // Iterate over the obstacle hypotheses
  for(std::size_t i = 0; i < obstacleHypotheses.size(); ++i)
  {
    // check whether the obstacle could be seen in the image
    GlobalOpponentsHypothesis* closer = &(obstacleHypotheses[i]);
    Vector2f centerInImage;

    // Continue with next obstacle if obstacle was seen in the last 300ms or is not in sight
    if(theFrameInfo.getTimeSince(closer->lastSeen) < recentlySeenTime || !closer->isBetween(cameraAngleLeft, cameraAngleRight) ||
      !closer->isInImage(centerInImage, theCameraInfo, theCameraMatrix))
      continue;

    COMPLEX_DRAWING("module:ObstacleModelProvider:obstacleNotSeen")
    {
      Vector2f leftInImage, rightInImage;
      if(Transformation::robotToImage(closer->left, theCameraMatrix, theCameraInfo, leftInImage))
        LARGE_DOT("module:ObstacleModelProvider:obstacleNotSeen", closer->left.x(), closer->left.y(), ColorRGBA::violet, ColorRGBA::black);
      LARGE_DOT("module:ObstacleModelProvider:obstacleNotSeen", centerInImage.x(), centerInImage.y(), ColorRGBA::violet, ColorRGBA::black);
      if(Transformation::robotToImage(closer->right, theCameraMatrix, theCameraInfo, rightInImage))
        LARGE_DOT("module:ObstacleModelProvider:obstacleNotSeen", rightInImage.x(), rightInImage.y(), ColorRGBA::violet, ColorRGBA::black);
    }

    // Increase notSeenButShouldSeen and continue with next obstacle if any other obstacle is in the shadow of the obstacle
    // or the field boundary is further as the obstacle
    if(isAnyObstacleInShadow(closer, i, cameraAngleLeft, cameraAngleRight) || (theFieldBoundary.isValid &&
      closer->isFieldBoundaryFurtherAsObstacle(theCameraInfo, theCameraMatrix, theImageCoordinateSystem, theFieldBoundary)))
    {
      closer->notSeenButShouldSeenCount += std::max(1u, notSeenThreshold / 10);
      continue;
    }

    // Obstacle is not seen but should be seen
    ++closer->notSeenButShouldSeenCount;
  }
}

bool GlobalOpponentsTracker::isAnyObstacleInShadow(GlobalOpponentsHypothesis* closer, const std::size_t i, const float cameraAngleLeft, const float cameraAngleRight)
{
  for(std::size_t j = obstacleHypotheses.size() - 1; j > i; --j)
  {
    GlobalOpponentsHypothesis* further = &(obstacleHypotheses[j]);

    // If the further obstacle was not seen, but is in sight.
    Vector2f centerInImage;
    if(further->lastSeen != theFrameInfo.time
      && further->isBetween(cameraAngleLeft, cameraAngleRight)
      && further->isInImage(centerInImage, theCameraInfo, theCameraMatrix))
    {
      // Swap further and closer if further obstacle is closer than closer obstacle
      if(further->center.squaredNorm() < closer->center.squaredNorm())
        std::swap(closer, further);

      // If the obstacle is not fallen and the further obstacle is behind the closer obstacle.
      if(closer->type < Obstacle::fallenSomeRobot && further->isBehind(*closer))
        return true;
    }
  }
  return false;
}

bool GlobalOpponentsTracker::shouldIgnore(const GlobalOpponentsHypothesis& obstacle) const
{
  if(goalAreaIgnoreTolerance == 0.f ||
    !theGameState.kickOffSetupFromTouchlines ||
    !theGameState.isGoalkeeper())
    return false;
  else
  {
    const Vector2f obstacleInField = theRobotPose * obstacle.center;
    return obstacleInField.x() < theFieldDimensions.xPosOwnGoalArea + goalAreaIgnoreTolerance &&
      obstacleInField.y() < theFieldDimensions.yPosLeftGoalArea + goalAreaIgnoreTolerance &&
      obstacleInField.y() > theFieldDimensions.yPosRightGoalArea - goalAreaIgnoreTolerance;
  }
}

void GlobalOpponentsTracker::draw()
{
  // The zones that will get a special handling:
  COMPLEX_DRAWING("module:GlobalOpponentsTracker:penaltyZones")
  {
    for(const auto& rect : returnFromPenaltyZonesOpponentTeam)
    {
      RECTANGLE("module:GlobalOpponentsTracker:penaltyZones", rect.a.x(), rect.a.y(), rect.b.x(), rect.b.y(), 10, Drawings::solidPen, ColorRGBA(200, 200, 200));
    }
    for(const auto& rect : penalizedRobotZonesOpponentTeam)
    {
      RECTANGLE("module:GlobalOpponentsTracker:penaltyZones", rect.a.x(), rect.a.y(), rect.b.x(), rect.b.y(), 10, Drawings::solidPen, ColorRGBA(200, 200, 200));
    }
  }
}
