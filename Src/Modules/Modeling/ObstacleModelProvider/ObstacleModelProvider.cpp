/**
 * @file ObstacleModelProvider.cpp
 *
 * This is the implementation of a module that combines arm contacts, foot bumper contacts and
 * obstacle percepts into one obstacle model.
 *
 * @author Florian Maaß
 * @author Jan Fiedler & Nicole Schrader
 */

#include "ObstacleModelProvider.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/Measurements.h"

MAKE_MODULE(ObstacleModelProvider, modeling);

void ObstacleModelProvider::update(ObstacleModel& obstacleModel)
{
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:maxDistance", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:merge", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:changeTeam", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:cameraAngle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:obstacleNotSeen", "drawingOnImage");

  CIRCLE("module:ObstacleModelProvider:maxDistance", theRobotPose.translation.x(), theRobotPose.translation.y(), maxDistance, 6, Drawings::dottedPen,
         ColorRGBA::black, Drawings::noBrush, ColorRGBA::black);

  if(clearAndFinish(obstacleModel))
    return;

  deleteObstacles(); // Delete old obstacles and obstacles that are no longer hypotheses.
  dynamic(); // Apply extended kalman filter prediction step to hypotheses.
  if(useArmContactModel)
    addArmContacts(); // Add hypotheses measured by arm contact.
  if(useFootBumperState)
    addFootContacts(); // Add hypotheses measured by foot contact.
  addPlayerPercepts(); // Add players field percepts.
  if(useTeammatePositionForClassification)
    considerTeamData(); // Fit team and position of obstacles located exclusively near a team member.
  mergeOverlapping(); // Overlapping hypotheses are merged together.

  // Set the same length for left and right.
  for(auto& obstacle : obstacleHypotheses)
    obstacle.setLeftRight((obstacle.left - obstacle.right).norm() * .5f);

  shouldBeSeen(); // Mark obstacles that should be seen but wasn't seen recently.
  STOPWATCH("ObstacleModel:calculateVelocity")
    calculateVelocity(); // Calculate velocity.
  // Update obstacles from valid hypotheses.
  obstacleModel.obstacles.clear();
  for(const auto& ob : obstacleHypotheses)
  {
    if(isObstacle(ob))
      obstacleModel.obstacles.emplace_back(ob);
  }
}

bool ObstacleModelProvider::clearAndFinish(ObstacleModel& obstacleModel)
{
  DEBUG_RESPONSE_ONCE("module:ObstacleModelProvider:clear")
    obstacleHypotheses.clear();

  if(theRobotInfo.penalty != PENALTY_NONE
     || theGameInfo.state == STATE_INITIAL
     // While falling down / getting up / the obstacles might be invalid, better clean up
     || theFallDownState.state == FallDownState::falling
     || theFallDownState.state == FallDownState::fallen
     || theMotionInfo.executedPhase == MotionPhase::getUp
     || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) // Penalty shootout -> obstacles will be ignored
  {
    if(theGameInfo.state != STATE_FINISHED) // If the GameController operator fails epically and resets from finished to playing
    {
      obstacleModel.obstacles.clear();
      obstacleHypotheses.clear();
    }
    return true;
  }
  return false;
}

void ObstacleModelProvider::deleteObstacles()
{
  for(auto obstacle = obstacleHypotheses.begin(); obstacle != obstacleHypotheses.end();)
  {
    const float obstacleRadius = obstacle->type == Obstacle::goalpost ? theFieldDimensions.goalPostRadius : Obstacle::getRobotDepth();
    const float centerDistanceSquared = obstacle->center.squaredNorm();
    Vector2f absObsPos = Transformation::robotToField(theRobotPose, obstacle->center);
    if(obstacle->notSeenButShouldSeenCount >= notSeenThreshold
       || theFrameInfo.getTimeSince(obstacle->lastSeen) >= deleteAfter
       || centerDistanceSquared >= sqr(maxDistance)
       || centerDistanceSquared <= sqr(obstacleRadius * 0.5f) // Obstacle is really inside us
       // HACK: Ignore the referee hand before the kick-off.
       || (theExtendedGameInfo.gameStateLastFrame != STATE_PLAYING && theGameInfo.state == STATE_PLAYING && theFrameInfo.getTimeSince(obstacle->lastSeen) > 1500)
       || theFieldDimensions.clipToField(absObsPos) > 500.f // obstacleIsNotOnField
       //|| obstacle->velocity.squaredNorm() > sqr(maxVelocity)  // The velocity is currently not accurate enough for this
      )
    {
      obstacle = obstacleHypotheses.erase(obstacle);
    }
    else
      ++obstacle;
  }
}

void ObstacleModelProvider::dynamic()
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

  for(ObstacleHypothesis& obstacle : obstacleHypotheses)
    obstacle.dynamic(odometryRotation, odometryTranslation, odometryJacobian, odometryNoiseX, odometryNoiseY);
}

void ObstacleModelProvider::addArmContacts()
{
  merged.clear();
  merged.resize(obstacleHypotheses.size());

  FOREACH_ENUM(Arms::Arm, arm)
  {
    if(theArmContactModel.status[arm].contact && theFrameInfo.getTimeSince(theArmContactModel.status[arm].timeOfLastContact) <= maxContactTime)
    {
      if(!armContact[arm])
      {
        ANNOTATION("ObstacleModelProvider", TypeRegistry::getEnumName(arm) << "ArmContact");
        armContact[arm] = true;
      }
      Vector2f center = (theTorsoMatrix.inverse() * theRobotModel.limbs[Limbs::combine(arm, Limbs::shoulder)].translation).topRows(2);
      center.y() += sgn(center.y()) * (Obstacle::getRobotDepth() + 15.f);
      ObstacleHypothesis obstacle(armCov, center, Vector2f::Zero(), Vector2f::Zero(), theFrameInfo.time, Obstacle::unknown, 1);
      obstacle.setLeftRight(Obstacle::getRobotDepth());
      // Insert valid obstacle.
      tryToMerge(obstacle);
    }
    else
      armContact[arm] = false;
  };
}

void ObstacleModelProvider::addFootContacts()
{
  merged.clear();
  merged.resize(obstacleHypotheses.size());

  FOREACH_ENUM(Legs::Leg, leg)
  {
    if(theFootBumperState.status[leg].contact && theFrameInfo.getTimeSince(theFootBumperState.status[leg].lastContact) <= maxContactTime)
    {
      if(!footContact[leg])
      {
        ANNOTATION("ObstacleModelProvider", TypeRegistry::getEnumName(leg) << "FootContact.");
        footContact[leg] = true;
      }
      Vector2f center = (theTorsoMatrix.inverse() * theRobotModel.limbs[Limbs::combine(leg, Limbs::foot)].translation).topRows(2);
      center.x() += Obstacle::getRobotDepth() + distJointToToe + distToeToBumper;
      ObstacleHypothesis obstacle(feetCov, center, Vector2f::Zero(), Vector2f::Zero(), theFrameInfo.time, Obstacle::unknown, 1);
      obstacle.setLeftRight(Obstacle::getRobotDepth());
      // Insert valid obstacle.
      tryToMerge(obstacle);
    }
    else
      footContact[leg] = false;
  };
}

void ObstacleModelProvider::addPlayerPercepts()
{
  if(theObstaclesFieldPercept.obstacles.empty())
    return;

  merged.clear();
  merged.resize(obstacleHypotheses.size());

  const Vector2f& robotRotationDeviation = theMotionInfo.executedPhase == MotionPhase::stand ? pRobotRotationDeviationInStand : pRobotRotationDeviation;

  for(const auto& percept : theObstaclesFieldPercept.obstacles)
  {
    // To far away?
    if(percept.center.squaredNorm() >= sqr(maxDistance))
      continue;

    Obstacle::Type type = percept.type == ObstaclesFieldPercept::opponentPlayer ? (percept.fallen ? Obstacle::fallenOpponent : Obstacle::opponent)
                          : (percept.type == ObstaclesFieldPercept::ownPlayer ? (percept.fallen ? Obstacle::fallenTeammate : Obstacle::teammate)
                             : (percept.fallen ? Obstacle::fallenSomeRobot : Obstacle::someRobot));

    const Matrix2f& cov = Measurements::positionToCovarianceMatrixInRobotCoordinates(percept.center, 0.f, theCameraMatrix, theCameraMatrix.inverse(), robotRotationDeviation);
    ObstacleHypothesis obstacle(cov, percept.center,
                                percept.left.normalized(percept.left.norm() + Obstacle::getRobotDepth()),
                                percept.right.normalized(percept.right.norm() + Obstacle::getRobotDepth()), theFrameInfo.time, type, 1);

    // Obstacles have a minimum size
    if((obstacle.left - obstacle.right).squaredNorm() < sqr(2 * Obstacle::getRobotDepth()))
      obstacle.setLeftRight(Obstacle::getRobotDepth());
    tryToMerge(obstacle);
  }
}

void ObstacleModelProvider::tryToMerge(const ObstacleHypothesis& measurement)
{
  if(obstacleHypotheses.empty())
  {
    obstacleHypotheses.emplace_back(measurement);
    merged.push_back(true);
    return;
  }

  const float mergeDistanceSquared = sqr(calculateMergeRadius(measurement.center, measurement.type, maxMergeRadius));
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

    obstacleHypotheses[atMerge].measurement(measurement, weightedSum, theFieldDimensions.goalPostRadius); // EKF
    obstacleHypotheses[atMerge].considerType(measurement, teamThreshold, uprightThreshold);
    obstacleHypotheses[atMerge].seenCount += measurement.seenCount;
    obstacleHypotheses[atMerge].notSeenButShouldSeenCount = 0; // Reset that counter.
    merged[atMerge] = true;
    return;
  }

  // Did not find possible match.
  obstacleHypotheses.emplace_back(measurement);
  merged.push_back(true);
}

void ObstacleModelProvider::considerTeamData()
{
  // TODO try to add teammate.theBehaviorStatus.walkingTo to track teammates better. Or use to adapt velocity.
  if(obstacleHypotheses.empty() || theTeamData.teammates.empty())
    return;

  for(const Teammate& teammate : theTeamData.teammates)
  {
    // Only for one frame (upper and lower) after receiving a package.
    if(teammate.status != Teammate::PLAYING
       || theFrameInfo.getTimeSince(teammate.timeWhenLastPacketReceived) > 2 * static_cast<int>(mergeOverlapTimeDiff))
      continue;

    const Matrix2f cov = teammate.theRobotPose.covariance.topLeftCorner(2, 2);
    ASSERT(cov(0, 1) == cov(1, 0));
    const Vector2f relativePosition = Transformation::fieldToRobot(theRobotPose, teammate.theRobotPose.translation);

    // Only consider teammates that are located nearby.
    if(relativePosition.squaredNorm() > sqr(maxDistance))
      continue;

    ObstacleHypothesis teammateHypothesis(cov, relativePosition, Vector2f::Zero(), Vector2f::Zero(),
                                          teammate.timeWhenLastPacketReceived,
                                          Obstacle::teammate, minPercepts / 2);
    teammateHypothesis.setLeftRight(Obstacle::getRobotDepth());
    // Strengthens the hypothesis that this is a teammate.
    teammateHypothesis.team = -2;

    const float mergeRadius = calculateMergeRadius(teammateHypothesis.center, teammateHypothesis.type, maxTeammateRadius);
    CIRCLE("module:ObstacleModelProvider:changeTeam", teammateHypothesis.center.x(), teammateHypothesis.center.y(), mergeRadius, 5, Drawings::dashedPen, ColorRGBA::cyan, Drawings::noBrush, ColorRGBA::cyan);

    std::size_t atMerge = std::numeric_limits<std::size_t>::max(); // Element matching the merge condition.
    for(std::size_t i = 0; i < obstacleHypotheses.size(); ++i)
    {
      const float distanceSquared = (teammateHypothesis.center - obstacleHypotheses[i].center).squaredNorm();

      if(distanceSquared < sqr(mergeRadius))
      {
        if(atMerge < std::numeric_limits<std::size_t>::max())
        {
          atMerge = std::numeric_limits<std::size_t>::max();
          break;
        }
        atMerge = i;
      }
    }

    if(atMerge < std::numeric_limits<std::size_t>::max())
    {
      COMPLEX_DRAWING("module:ObstacleModelProvider:changeTeam")
      {
        if(obstacleHypotheses[atMerge].seenCount >= minPercepts)
          CIRCLE("module:ObstacleModelProvider:changeTeam", obstacleHypotheses[atMerge].center.x(), obstacleHypotheses[atMerge].center.y(), (obstacleHypotheses[atMerge].left - obstacleHypotheses[atMerge].right).norm() * .25f, 10, Drawings::dashedPen, ColorRGBA::magenta, Drawings::solidPen, ColorRGBA::magenta);
      }
      Obstacle::fusion2D(obstacleHypotheses[atMerge], teammateHypothesis);
      // Since fusion2D makes all previous positions unusable for a correct calculation.
      obstacleHypotheses[atMerge].lastObservations.clear();
      obstacleHypotheses[atMerge].considerType(teammateHypothesis, teamThreshold, uprightThreshold);
      obstacleHypotheses[atMerge].lastSeen = std::max(obstacleHypotheses[atMerge].lastSeen, teammateHypothesis.lastSeen);
    }
  }
}

void ObstacleModelProvider::mergeOverlapping()
{
  // TODO The merge with velocity is still missing here. The reason for this is that tests are necessary to see how precise it is.
  // If this is deemed to be sufficient, rows of obstacles with robots running in parallel can be prevented.
  // In addition, the maximum velocity, which is valid, is not yet handled.
  if(obstacleHypotheses.size() < 2)
    return;

  for(std::size_t i = 0; i < obstacleHypotheses.size(); ++i)
  {
    ObstacleHypothesis& actual = obstacleHypotheses[i];
    for(std::size_t j = obstacleHypotheses.size() - 1; j > i; --j)
    {
      const ObstacleHypothesis& other = obstacleHypotheses[j];

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
         && (actual.type == Obstacle::unknown || actual.type == Obstacle::someRobot || actual.type == Obstacle::fallenSomeRobot
             || other.type == Obstacle::unknown || other.type == Obstacle::someRobot || other.type == Obstacle::fallenSomeRobot
             || actual.type == other.type)) // Their type is unknown, someRobot or fallenSomeRobot or their type is equal
      {
        Obstacle::fusion2D(actual, other);
        // Since fusion2D makes all previous positions unusable for a correct calculation.
        actual.lastObservations.clear();
        if(actual.type == Obstacle::goalpost)
          actual.setLeftRight(theFieldDimensions.goalPostRadius);
        actual.considerType(other, teamThreshold, uprightThreshold);
        actual.lastSeen = std::max(actual.lastSeen, other.lastSeen);
        actual.seenCount = std::max(actual.seenCount, other.seenCount);
        actual.notSeenButShouldSeenCount = (actual.notSeenButShouldSeenCount + other.notSeenButShouldSeenCount) / 2;
        obstacleHypotheses.erase(obstacleHypotheses.begin() + j);
      }
    }
  }
}

void ObstacleModelProvider::shouldBeSeen()
{
  if(obstacleHypotheses.empty())
    return;

  const float cameraAngle = theCameraMatrix.rotation.getZAngle();
  const float cameraAngleLeft = cameraAngle + theCameraInfo.openingAngleWidth * cameraAngleFactor,
              cameraAngleRight = cameraAngle - theCameraInfo.openingAngleWidth * cameraAngleFactor;

  COMPLEX_DRAWING("module:ObstacleModelProvider:cameraAngle")
  {
    // The camera angle being used
    Vector2f camLeft(static_cast<float>(maxDistance), 0.f);
    Vector2f camRight(static_cast<float>(maxDistance), 0.f);
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
    ObstacleHypothesis* closer = &(obstacleHypotheses[i]);
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

bool ObstacleModelProvider::isAnyObstacleInShadow(ObstacleHypothesis* closer, const std::size_t i, const float cameraAngleLeft, const float cameraAngleRight)
{
  for(std::size_t j = obstacleHypotheses.size() - 1; j > i; --j)
  {
    ObstacleHypothesis* further = &(obstacleHypotheses[j]);

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

void ObstacleModelProvider::calculateVelocity()
{
  for(auto& obstacle : obstacleHypotheses)
  {
    // Add obstacle position and timestamp to ring buffer every 300 milliseconds. Should be nothing too short to avoid backward problems.
    if(obstacle.lastObservations.empty() || theFrameInfo.getTimeSince(obstacle.lastObservations.front().timestamp) > velocitySampleTime)
    {
      obstacle.lastObservations.push_front(Observation(theFrameInfo.time, obstacle.center, obstacle.covariance));

      // Calculate only if there is enough data and it is really an obstacle.
      if(obstacle.lastObservations.size() >= minSamplesForCalculation && isObstacle(obstacle))
      {
        // Calculates deviation of static obstacle.
        const float stdDevStatic = obstacle.calculateStdDevOfStaticObstacleHypothesis();

        // Calculates deviation of moving obstacle.
        Vector2f velocity;
        Matrix2f velocityCovariance;
        const float stdDevMoving = obstacle.calculateStdDevOfMovingObstacleHypothesis(velocity, velocityCovariance);

        const float speed = velocity.norm();
        if(stdDevMoving < stdDevStatic && speed > minVelocityForMotionDetection && speed < maxVelocityForMotionDetection)
        {
          // const Vector2f& absObsPos = Transformation::robotToField(theRobotPose, obstacle.center);
          // ANNOTATION("MovingObstacle", "Moving Obstacle(" << absObsPos.x() << "," << absObsPos.y() << "): " << stdDevMoving << " < " << stdDevStatic << "  with " << speed << "mm/s!");
          // OUTPUT_TEXT("Moving Obstacle(" << absObsPos.x() << "," << absObsPos.y() << "): " << stdDevMoving << " < " << stdDevStatic << "  with " << speed << "mm/s!");

          obstacle.velocity = velocity;
          obstacle.velocityCovariance = velocityCovariance;
        }
        else
        {
          obstacle.velocity = Vector2f::Zero();
          obstacle.velocityCovariance = Matrix2f::Identity();
        }
      }
    }
  }
}
