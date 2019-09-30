/**
 * @file ObstacleModelProvider.cpp
 *
 * This is the implementation of a module that combines arm contacts, foot bumper contacts and
 * obstacle percepts (from the module PlayersPerceptor) into one obstacle model.
 *
 * @author <a href="mailto:flomaass@informatik.uni-bremen.de">Florian Maaß</a>
 */

#include "ObstacleModelProvider.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/Measurements.h"
#include "Tools/Modeling/Obstacle.h"
#include <algorithm>
#include <limits>
#include <set>

MAKE_MODULE(ObstacleModelProvider, modeling)

#define _STOPWATCH(name, ...) STOPWATCH("ObstacleModelProvider:" #name) name(__VA_ARGS__)

void ObstacleModelProvider::update(ObstacleModel& obstacleModel)
{
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:maxDistance", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:merge", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:cameraAngle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:obstacleNotSeen", "drawingOnImage");

  CIRCLE("module:ObstacleModelProvider:maxDistance", theRobotPose.translation.x(), theRobotPose.translation.y(), maxDistance, 6, Drawings::dottedPen,
         ColorRGBA::black, Drawings::noBrush, ColorRGBA::black);

  if(clearAndFinish(obstacleModel))
    return;

  _STOPWATCH(deleteObstacles); // Delete old obstacles and obstacles that are no longer hypotheses.
  _STOPWATCH(dynamic); // Apply extended kalman filter prediction step to hypotheses.
  _STOPWATCH(addArmContacts); // Add hypotheses measured by arm contact.
  _STOPWATCH(addFootContacts); // Add hypotheses measured by foot contact.
  _STOPWATCH(addPlayerPercepts); // Add players field percepts.
  _STOPWATCH(considerTeammates); // correct whether teammate or opponent player by given pose
  _STOPWATCH(mergeOverlapping); // Overlapping hypotheses are merged together.
  _STOPWATCH(setLeftRight); // orthogonal obstacle.left/right
  _STOPWATCH(shouldBeSeen); // obstacles that are obviously not present will be deleted
  _STOPWATCH(propagateObstacles, obstacleModel); // Update obstacles fram valid hypotheses.

  BH_TRACE;
}

bool ObstacleModelProvider::clearAndFinish(ObstacleModel& obstacleModel)
{
  DEBUG_RESPONSE_ONCE("module:ObstacleModelProvider:clear")
    obstacleHypotheses.clear();

  if(theRobotInfo.penalty != PENALTY_NONE
     || theGameInfo.state == STATE_INITIAL
     //while falling down / getting up / the obstacles might be invalid, better clean up
     || theFallDownState.state == FallDownState::falling
     || theFallDownState.state == FallDownState::fallen
     || theMotionInfo.motion == MotionRequest::getUp
     || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) //penalty shootout -> obstacles will be ignored
  {
    if(theGameInfo.state != STATE_FINISHED) //if the gamecontroller operator fails epicly and resets from finished to playing
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
    if(obstacle->notSeenButShouldSeenCount >= notSeenThreshold
       || theFrameInfo.getTimeSince(obstacle->lastSeen) >= deleteAfter
       || centerDistanceSquared >= sqr(maxDistance)
       || centerDistanceSquared <= sqr(obstacleRadius * .5f) //obstacle is really inside us
       || (theCognitionStateChanges.lastGameState != STATE_PLAYING && theGameInfo.state == STATE_PLAYING && theFrameInfo.getTimeSince(obstacle->lastSeen) > 1500)
       || obstacleIsNotOnField(*obstacle)
       //|| obstacle->velocity.squaredNorm() > sqr(maxVelocity)  //velocity is deactivated
      )
    {
      obstacle = obstacleHypotheses.erase(obstacle);
    }
    else
      ++obstacle;
  }
}

bool ObstacleModelProvider::obstacleIsNotOnField(const ObstacleHypothesis& obstacle) const
{
  Vector2f absObsPos = Transformation::robotToField(theWorldModelPrediction.robotPose, obstacle.center);
  return theFieldDimensions.clipToField(absObsPos) > 500.f;;
}

void ObstacleModelProvider::dynamic()
{
  const float odometryRotation = -theOdometer.odometryOffset.rotation; //obstacle has to move in the opposite direction
  const Vector2f odometryTranslation = -theOdometer.odometryOffset.translation.rotated(-theOdometer.odometryOffset.rotation);
  Matrix2f odometryJacobian;
  odometryJacobian << std::cos(odometryRotation), -std::sin(odometryRotation), std::sin(odometryRotation), std::cos(odometryRotation);
  //noise
  //todo add noise from rotation
  const float odometryDeviationX = sqr(odometryTranslation.x() * odoDeviation.x());
  const float odometryDeviationY = sqr(odometryTranslation.y() * odoDeviation.y());
  //process noise
  const float odometryNoiseX = sqr(pNp) + odometryDeviationX;
  const float odometryNoiseY = sqr(pNp) + odometryDeviationY;

  for(auto& obstacle : obstacleHypotheses)
    obstacle.dynamic(odometryRotation, odometryTranslation, odometryJacobian, odometryNoiseX, odometryNoiseY);
}

void ObstacleModelProvider::addArmContacts()
{
  if(!useArmContactModel)
    return;

  merged.clear();
  merged.resize(obstacleHypotheses.size());

  auto addArmContact = [&](const Arms::Arm arm)
  {
    if(theArmContactModel.status[arm].contact && theFrameInfo.getTimeSince(theArmContactModel.status[arm].timeOfLastContact) <= 2000)
    {
      if(!armContact[arm])
      {
        ANNOTATION("ObstacleModelProvider", TypeRegistry::getEnumName(arm) << "ArmContact");
        armContact[arm] = true;
      }
      Vector2f center = (theTorsoMatrix.inverse() * theRobotModel.limbs[Limbs::combine(arm, Limbs::shoulder)].translation).topRows(2);
      center.y() += sgn(center.y()) * (Obstacle::getRobotDepth() + 15.f); // fehlt hier nicht torso inv?
      ObstacleHypothesis obstacle(armCov, center, Vector2f::Zero(), Vector2f::Zero(), theFrameInfo.time, 1, Obstacle::unknown);
      obstacle.setLeftRight(Obstacle::getRobotDepth());
      //Insert valid obstacle
      tryToMerge(obstacle);
    }
    else
      armContact[arm] = false;
  };

  FOREACH_ENUM(Arms::Arm, arm)
    addArmContact(arm);
}

void ObstacleModelProvider::addFootContacts()
{
  if(!useFootBumperState)
    return;

  merged.clear();
  merged.resize(obstacleHypotheses.size());

  auto addFootBumper = [&](const Legs::Leg leg)
  {
    if(theFootBumperState.status[leg].contact && theFrameInfo.getTimeSince(theFootBumperState.status[leg].lastContact) <= 2000)
    {
      if(!footContact[leg])
      {
        ANNOTATION("ObstacleModelProvider", TypeRegistry::getEnumName(leg) << "FootContact.");
        footContact[leg] = true;
      }
      Vector2f center = (theTorsoMatrix.inverse() * theRobotModel.limbs[Limbs::combine(leg, Limbs::foot)].translation).topRows(2);
      center.x() += Obstacle::getRobotDepth() + 65.f + 15.f; //65mm should be the distance from the joint to the toe 65mm is the distance between the bumper and the joint
      ObstacleHypothesis obstacle(feetCov, center, Vector2f::Zero(), Vector2f::Zero(), theFrameInfo.time, 1, Obstacle::unknown);
      obstacle.setLeftRight(Obstacle::getRobotDepth());
      //Insert valid obstacle
      tryToMerge(obstacle);
    }
    else
      footContact[leg] = false;
  };

  FOREACH_ENUM(Legs::Leg, leg)
    addFootBumper(leg);
}

void ObstacleModelProvider::addPlayerPercepts()
{
  if(theObstaclesFieldPercept.obstacles.empty())
    return;

  merged.clear();
  merged.resize(obstacleHypotheses.size());

  const Vector2f& robotRotationDeviation = theMotionInfo.isStanding() ? pRobotRotationDeviationInStand : pRobotRotationDeviation;

  for(const auto& percept : theObstaclesFieldPercept.obstacles)
  {
    if(percept.center.squaredNorm() >= sqr(maxDistance))
      continue;

    Obstacle::Type t = Obstacle::someRobot;
    if(percept.type == ObstaclesFieldPercept::opponentPlayer)
      t = percept.fallen ? Obstacle::fallenOpponent : Obstacle::opponent;
    else if(percept.type == ObstaclesFieldPercept::ownPlayer)
      t = percept.fallen ? Obstacle::fallenTeammate : Obstacle::teammate;
    else if(percept.fallen)
      t = Obstacle::fallenSomeRobot;

    const Matrix2f cov = Measurements::positionToCovarianceMatrixInRobotCoordinates(percept.center, 0.f, theCameraMatrix, theCameraMatrix.inverse(), robotRotationDeviation);
    ObstacleHypothesis obstacle(cov, percept.center,
                                percept.left.normalized(percept.left.norm() + Obstacle::getRobotDepth()),
                                percept.right.normalized(percept.right.norm() + Obstacle::getRobotDepth()), theFrameInfo.time, 1, t);
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
  float distanceSquared = 0.f;
  const float thisMergeDistance = measurement.type == Obstacle::goalpost ? goalMergeDistance : mergeDistance;
  const float mergeBonsu = measurement.center.norm() / 1000.f; // Removed square. A.H.
  // This is the old formula. I do not like it and it leads to "obstacle spraying". T.L.
  //const float robotDepths = (mergeBonsu >= 2.f && mergeBonsu <= 6.f ? std::floor(mergeBonsu) : 0.f) * Obstacle::getRobotDepth(); //for every meter there is a bonus of a robot radius in mm
  // Improved formula. This is not a solution but appears to work much better on the robots. T.L.
  const float robotDepths = (mergeBonsu >= 1.5f ? mergeBonsu : 0.f) * Obstacle::getRobotDepth() * 2.5f; //for every meter there is a bonus of 2.5 robot radii in mm

  float possibleMergeDistSquared = std::numeric_limits<float>::max();
  size_t atMerge = 0; //element matching the merge condition
  for(size_t i = 0; i < obstacleHypotheses.size(); ++i)
  {
    if(merged[i])
      continue;

    const float mergeDistanceSquared = sqr(robotDepths + thisMergeDistance);

    distanceSquared = (measurement.center - obstacleHypotheses[i].center).squaredNorm();
    if(distanceSquared <= mergeDistanceSquared && distanceSquared <= possibleMergeDistSquared) //found probably matching obstacle
    {
      possibleMergeDistSquared = distanceSquared;
      atMerge = i;
    }
  }

  if(possibleMergeDistSquared < std::numeric_limits<float>::max())
  {
    LINE("module:ObstacleModelProvider:merge", measurement.center.x(), measurement.center.y(),
         obstacleHypotheses[atMerge].center.x(), obstacleHypotheses[atMerge].center.y(), 10, Drawings::dashedPen, ColorRGBA::red);
    obstacleHypotheses[atMerge].measurement(measurement, weightedSum, theFieldDimensions); //EKF
    obstacleHypotheses[atMerge].considerType(measurement, colorThreshold, uprightThreshold);
    obstacleHypotheses[atMerge].lastSeen = measurement.lastSeen;
    obstacleHypotheses[atMerge].seenCount += measurement.seenCount;
    obstacleHypotheses[atMerge].notSeenButShouldSeenCount = 0; //reset that counter
    merged[atMerge] = true;
    return;
  }
  //did not found possible match
  obstacleHypotheses.emplace_back(measurement);
  merged.push_back(true);
}

void ObstacleModelProvider::considerTeammates()
{
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:teammates", "drawingOnField");
  if(!useTeammatePositionForClassification)
    return;

  //don't risk false classification if no communication is possibles
  if(obstacleHypotheses.empty() || theTeamData.teammates.empty())
    return;

  size_t neverSeen = 0; //maybe never seen teammates

  std::set<size_t> mergedTeammates;
  for(const auto& teammate : theTeamData.teammates)
  {
    if(teammate.status != Teammate::PLAYING)
      continue;

    const Matrix2f cov = teammate.theRobotPose.covariance.topLeftCorner(2, 2);
    ASSERT(cov(0, 1) == cov(1, 0));
    const Vector2f relativePosition = theRobotPose.inversePose * teammate.theRobotPose.translation;
    CIRCLE("module:ObstacleModelProvider:teammates", teammate.theRobotPose.translation.x(), teammate.theRobotPose.translation.y(), 100, 3, Drawings::dottedPen, ColorRGBA::black, Drawings::noBrush, ColorRGBA::yellow);
    if(relativePosition.squaredNorm() > sqr(maxDistance))
    {
      ++neverSeen;
      continue;
    }
    ObstacleHypothesis measurement(cov, relativePosition, Vector2f::Zero(), Vector2f::Zero(),
                                   teammate.timeWhenLastPacketReceived, minPercepts / 2,
                                   Obstacle::teammate);
    measurement.setLeftRight(Obstacle::getRobotDepth());
    //if the teammate is found in the local obstacle model, one would like to make sure, it is not classified as an opponent
    measurement.color = (colorThreshold / 2) * sgn(measurement.color);
    float distanceSquared = 0.f;
    float possibleMergeDistSquared = std::numeric_limits<float>::max();
    size_t atMerge = 0; //element matching the merge condition
    for(size_t i = 0; i < obstacleHypotheses.size(); ++i)
    {
      if(mergedTeammates.find(i) != mergedTeammates.end())
        continue;
      const float mergeBonsu = measurement.center.norm() / 1000.f;
      const float robotDepths = (mergeBonsu >= 1.5f ? mergeBonsu : 0.f) * Obstacle::getRobotDepth() * 2.5f; //for every meter there is a bonus of 2.5 robot radii in mm
      const float mergeDistanceSquared = sqr(robotDepths + mergeDistance);
      distanceSquared = (measurement.center - obstacleHypotheses[i].center).squaredNorm();
      if(distanceSquared <= mergeDistanceSquared && distanceSquared <= possibleMergeDistSquared) //found probably matching obstacle
      {
        possibleMergeDistSquared = distanceSquared;
        atMerge = i;
      }
    }
    if(possibleMergeDistSquared < std::numeric_limits<float>::max())
    {
      obstacleHypotheses[atMerge].considerType(measurement, colorThreshold, uprightThreshold);
      obstacleHypotheses[atMerge].lastSeen = theFrameInfo.time;
      mergedTeammates.insert(atMerge);
    }
    else if(relativePosition.x() < -300.f || distanceSquared > sqr(maxDistance)) //magic number; teammate is behind me or too far away
      ++neverSeen;
  }
  //found all my teammates? other robots should be in the other team
  if(mergedTeammates.size() + neverSeen != theTeamData.teammates.size())
    return;
  ObstacleHypothesis opponent(Obstacle::opponent);
  opponent.upright = 0;
  for(size_t i = 0; i < obstacleHypotheses.size(); ++i)
  {
    if(mergedTeammates.find(i) != mergedTeammates.end())
    {
      mergedTeammates.erase(i);
      continue;
    }
    if(obstacleHypotheses[i].type > Obstacle::unknown)
      obstacleHypotheses[i].considerType(opponent, colorThreshold, uprightThreshold);
  }
}

void ObstacleModelProvider::mergeOverlapping()
{
  if(obstacleHypotheses.size() < 2)
    return;

  for(size_t i = 0; i < obstacleHypotheses.size(); ++i)
  {
    for(size_t j = obstacleHypotheses.size() - 1; j > i; --j)
    {
      // obstacles that have been seen too close on the time axis should not be merged (for whatever reasons)
      if(std::max(obstacleHypotheses[i].lastSeen, obstacleHypotheses[j].lastSeen) - std::min(obstacleHypotheses[i].lastSeen, obstacleHypotheses[j].lastSeen) < mergeOverlapTimeDiff)
        continue;

      const float overlapSquared = ((obstacleHypotheses[i].left - obstacleHypotheses[i].right) * .5f).squaredNorm() + ((obstacleHypotheses[j].left - obstacleHypotheses[j].right) * .5f).squaredNorm();
      const float distanceOfCentersSquared = (obstacleHypotheses[j].center - obstacleHypotheses[i].center).squaredNorm();
      if((distanceOfCentersSquared <= overlapSquared || distanceOfCentersSquared < sqr(2 * Obstacle::getRobotDepth())) &&
         ((obstacleHypotheses[i].type >= Obstacle::unknown && obstacleHypotheses[j].type >= Obstacle::unknown)
          || obstacleHypotheses[i].type == obstacleHypotheses[j].type))
      {
        Obstacle::fusion2D(obstacleHypotheses[i], obstacleHypotheses[j]);
        if(obstacleHypotheses[i].type == Obstacle::goalpost)
          obstacleHypotheses[i].setLeftRight(theFieldDimensions.goalPostRadius);
        obstacleHypotheses[i].considerType(obstacleHypotheses[j], colorThreshold, uprightThreshold);
        obstacleHypotheses[i].lastSeen = std::max(obstacleHypotheses[i].lastSeen, obstacleHypotheses[j].lastSeen);
        obstacleHypotheses[i].seenCount = std::max(obstacleHypotheses[i].seenCount, obstacleHypotheses[j].seenCount);
        obstacleHypotheses[i].notSeenButShouldSeenCount = obstacleHypotheses[i].notSeenButShouldSeenCount + obstacleHypotheses[j].notSeenButShouldSeenCount / 2;
        obstacleHypotheses.erase(obstacleHypotheses.begin() + j);
      }
    }
  }
}

void ObstacleModelProvider::setLeftRight()
{
  for(auto& obstacle : obstacleHypotheses)
    obstacle.setLeftRight((obstacle.left - obstacle.right).norm() * .5f);
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
    Vector2f camLeft(static_cast<float>(maxDistance), 0.f);
    Vector2f camRight(static_cast<float>(maxDistance), 0.f);
    camLeft = camLeft.rotate(cameraAngleLeft);
    camRight = camRight.rotate(cameraAngleRight);
    const ColorRGBA cameraColor = theCameraInfo.camera == CameraInfo::upper ? ColorRGBA::blue : ColorRGBA::yellow;
    LINE("module:ObstacleModelProvider:cameraAngle", 0, 0, camLeft.x(), camLeft.y(), 10, Drawings::solidPen, cameraColor);
    LINE("module:ObstacleModelProvider:cameraAngle", 0, 0, camRight.x(), camRight.y(), 10, Drawings::solidPen, cameraColor);
  }

  for(size_t i = 0; i < obstacleHypotheses.size(); ++i)
  {
    //check whether the obstacle could be seen in the image
    ObstacleHypothesis* closer = &(obstacleHypotheses[i]);
    Vector2f centerInImage;
    //check if obstacle is not in sight and was not seen for 300ms
    if(theFrameInfo.getTimeSince(closer->lastSeen) < 300 || !closer->isBetween(cameraAngleLeft, cameraAngleRight) ||
       !closer->isInImage(centerInImage, theCameraInfo, theCameraMatrix))
      continue;

    COMPLEX_DRAWING("module:ObstacleModelProvider:obstacleNotSeen")
    {
      Vector2f leftInImage, rightInImage;
      if(Transformation::robotToImage(closer->left, theCameraMatrix, theCameraInfo, leftInImage))
        MID_DOT("module:ObstacleModelProvider:obstacleNotSeen", closer->left.x(), closer->left.y(), ColorRGBA::violet, ColorRGBA::black);
      MID_DOT("module:ObstacleModelProvider:obstacleNotSeen", centerInImage.x(), centerInImage.y(), ColorRGBA::violet, ColorRGBA::black);
      if(Transformation::robotToImage(closer->right, theCameraMatrix, theCameraInfo, rightInImage))
        MID_DOT("module:ObstacleModelProvider:obstacleNotSeen", rightInImage.x(), rightInImage.y(), ColorRGBA::violet, ColorRGBA::black);
    }

    //is another obstacle behind this one and the further one was measured
    if(shadowedRobots(closer, i, cameraAngleLeft, cameraAngleRight))
      continue;
    //is too much green in the robot, the fieldboundary cuts the robot
    if(theFieldBoundary.isValid &&
       obstacleHypotheses[i].fieldBoundaryFurtherAsObstacle(centerInImage, notSeenThreshold, theCameraInfo, theCameraMatrix, theImageCoordinateSystem, theFieldBoundary))
      continue;
    //obstacle is not seen
    ++closer->notSeenButShouldSeenCount;
  }
}

bool ObstacleModelProvider::shadowedRobots(ObstacleHypothesis* closer, const size_t i, const float cameraAngleLeft, const float cameraAngleRight)
{
  for(size_t j = obstacleHypotheses.size() - 1; j > i; --j)
  {
    ObstacleHypothesis* further = &(obstacleHypotheses[j]);
    if(further->lastSeen == theFrameInfo.time)
      continue;
    //check if obstacle is not in sight
    Vector2f centerInImage2;
    if(!further->isBetween(cameraAngleLeft, cameraAngleRight) || !further->isInImage(centerInImage2, theCameraInfo, theCameraMatrix))
      continue;
    //which obstacle is closer
    if(further->center.squaredNorm() < closer->center.squaredNorm())
      std::swap(closer, further);
    if(closer->type >= Obstacle::fallenSomeRobot) //fallen robots in front of another obstacle is okay
      continue;
    //closer one seems to be non existent
    if(further->isBehind(*closer))
    {
      closer->notSeenButShouldSeenCount += std::max(1u, notSeenThreshold / 10);
      return true;
    }
  }
  return false;
}

void ObstacleModelProvider::propagateObstacles(ObstacleModel& obstacleModel) const
{
  obstacleModel.obstacles.clear();
  for(const auto& ob : obstacleHypotheses)
  {
    if(ob.seenCount >= minPercepts || debug)
      obstacleModel.obstacles.emplace_back(ob);
  }
}
