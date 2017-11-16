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
#include "Tools/Modeling/Obstacle.h"
#include <algorithm>
#include <limits>
#include <set>

MAKE_MODULE(ObstacleModelProvider, modeling)

#define _STOPWATCH(name, ...) STOPWATCH("ObstacleModelProvider:" #name) name(__VA_ARGS__)
constexpr unsigned int frameTimeDiff = static_cast<unsigned int>(Constants::cognitionCycleTime * 1000.f);

ObstacleModelProvider::ObstacleModelProvider()
{
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:maxDistance", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:merge", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:cameraAngle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:obstacleNotSeen", "drawingOnImage");
}

void ObstacleModelProvider::update(ObstacleModel& obstacleModel)
{
  CIRCLE("module:ObstacleModelProvider:maxDistance", theRobotPose.translation.x(), theRobotPose.translation.y(), maxDistance, 6, Drawings::dottedPen,
         ColorRGBA::black, Drawings::noBrush, ColorRGBA::black);

  if(clearAndFinish(obstacleModel))
    return;

  _STOPWATCH(deleteObstacles); //delete old obstacles and obstacles that are no longer obstacles for reasons
  _STOPWATCH(dynamic); //apply extended kalman filter prediction step to obstacles
  _STOPWATCH(addArmContacts); //whether to us the arm contact model or not
  _STOPWATCH(addFootContacts); //whether to use the foot contact model or not
  _STOPWATCH(addPlayerPercepts); //try to merge (if not, add new obstacle) with obstacles
  _STOPWATCH(considerTeammates); //correct whether teammate or opponent player by given pose
  _STOPWATCH(mergeOverlapping); //overlapping obstacles are merged together
  _STOPWATCH(setLeftRight); //orthogonal obstacle.left/right
  _STOPWATCH(shouldBeSeen); //obstacles that are obviously not present will be deleted
  _STOPWATCH(propagateObstacles, obstacleModel); //put valid obstacles in representation

  BH_TRACE;
}

bool ObstacleModelProvider::clearAndFinish(ObstacleModel& obstacleModel)
{
  DEBUG_RESPONSE_ONCE("module:ObstacleModelProvider:clear")
    iWantToBeAnObstacle.clear();

  if(theRobotInfo.penalty != PENALTY_NONE
     || theGameInfo.state == STATE_INITIAL
     //while falling down / getting up / the obstacles might be invalid, better clean up
     || theFallDownState.state == FallDownState::falling
     || theFallDownState.state == FallDownState::fallen
     || theMotionInfo.motion == MotionRequest::getUp
     || theGameInfo.secondaryState == STATE2_PENALTYSHOOT) //penalty shootout -> obstacles will be ignored
  {
    if(theGameInfo.state != STATE_FINISHED) //if the gamecontroller operator fails epicly and resets from finished to playing
    {
      obstacleModel.obstacles.clear();
      iWantToBeAnObstacle.clear();
    }

    return true;
  }

  return false;
}

void ObstacleModelProvider::addArmContacts()
{
  if(!useArmContactModel)
    return;

  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());

  auto addArmContact = [&](const Arms::Arm arm)
  {
    if(theArmContactModel.status[arm].contact && theFrameInfo.getTimeSince(theArmContactModel.status[arm].timeOfLastContact) <= 2000)
    {
      if(!armContact[arm])
      {
        ANNOTATION("ObstacleModelProvider", Arms::getName(arm) << "ArmContact");
        armContact[arm] = true;
      }
      Vector2f center = (theTorsoMatrix.inverse() * theRobotModel.limbs[Limbs::combine(arm, Limbs::shoulder)].translation).topRows(2);
      center.y() += sgn(center.y()) * (Obstacle::getRobotDepth() + 15.f); // fehlt hier nicht torso inv?
      InternalObstacle obstacle = InternalObstacle(armCov, center, Vector2f::Zero(), Vector2f::Zero(), theFrameInfo.time, 1,
                                                   Obstacle::unknown);
      obstacle.setLeftRight(Obstacle::getRobotDepth());
      //Insert valid obstacle
      tryToMerge(obstacle);
    }
    else
      armContact[arm] = false;
  };

  FOREACH_ENUM((Arms) Arm, arm)
    addArmContact(arm);
}

void ObstacleModelProvider::addFootContacts()
{
  if(!useFootBumperState)
    return;

  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());

  auto addFootBumper = [&](const Legs::Leg leg)
  {
    if(theFootBumperState.status[leg].contact && theFrameInfo.getTimeSince(theFootBumperState.status[leg].lastContact) <= 2000)
    {
      if(!footContact[leg])
      {
        ANNOTATION("ObstacleModelProvider", Legs::getName(leg) << "FootContact.");
        footContact[leg] = true;
      }
      Vector2f center = (theTorsoMatrix.inverse() * theRobotModel.limbs[Limbs::combine(leg, Limbs::foot)].translation).topRows(2);
      center.x() += Obstacle::getRobotDepth() + 65.f + 15.f; //65mm should be the distance from the joint to the toe
      //65mm is the distance between the bumper and the joint
      InternalObstacle obstacle = InternalObstacle(feetCov, center, Vector2f::Zero(), Vector2f::Zero(), theFrameInfo.time,
                                                   1, Obstacle::unknown);
      obstacle.setLeftRight(Obstacle::getRobotDepth());
      //Insert valid obstacle
      tryToMerge(obstacle);
    }
    else
      footContact[leg] = false;
  };

  FOREACH_ENUM((Legs) Leg, leg)
    addFootBumper(leg);
}

void ObstacleModelProvider::addPlayerPercepts()
{
  if(thePlayersFieldPercept.players.empty())
    return;
  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());
  for(const auto& percept : thePlayersFieldPercept.players)
  {
    if(!percept.detectedFeet)
      continue;

    if(percept.centerOnField.squaredNorm() >= maxDistOnFieldSquared || percept.centerOnField.squaredNorm() >= sqr(maxDistance))
      continue;

    Obstacle::Type t = Obstacle::someRobot;
    /*ignoreJersey HACK, please remove after RoboCup*/
    bool ignoreJersey = (theCameraInfo.camera == CameraInfo::lower && ignoreJerseyLowerCam && percept.lowerCamera)
                        || (theCameraInfo.camera == CameraInfo::upper && ignoreJerseyUpperCam && !percept.lowerCamera);
    if(percept.detectedJersey && !percept.ownTeam && !ignoreJersey)
    {
      if(percept.fallen)
      {
        t = Obstacle::fallenOpponent;
      }
      else
      {
        t = Obstacle::opponent;
      }
    }
    else if(percept.detectedJersey && !ignoreJersey) //ignoreJersey HACK, please remove after RoboCup
    {
      if(percept.fallen)
      {
        t = Obstacle::fallenTeammate;
      }
      else
      {
        t = Obstacle::teammate;
      }
    }
    else if(percept.fallen)
    {
      t = Obstacle::fallenSomeRobot;
    }
    InternalObstacle obstacle(getCovOfPointInWorld(percept.centerOnField), percept.centerOnField,
                              percept.leftOnField.normalized(percept.leftOnField.norm() + Obstacle::getRobotDepth()),
                              percept.rightOnField.normalized(percept.rightOnField.norm() + Obstacle::getRobotDepth()), theFrameInfo.time, 1, t);

    // adding orientation information::
    obstacle.orientation = percept.orientation;
    obstacle.detectedOrientation = percept.detectedOrientation;

    // ::adding orientation information

    if((obstacle.left - obstacle.right).squaredNorm() < sqr(2 * Obstacle::getRobotDepth()))
      obstacle.setLeftRight(Obstacle::getRobotDepth());
    tryToMerge(obstacle);
  }
}

void ObstacleModelProvider::dynamic()
{
  odometryRotation = -theOdometer.odometryOffset.rotation; //obstacle has to move in the opposite direction
  odometryTranslation = theOdometer.odometryOffset.translation * -1.f;
  odometryJacobian << std::cos(odometryRotation), -std::sin(odometryRotation), std::sin(odometryRotation), std::cos(odometryRotation);
  //noise
  //todo add noise from rotation
  float odometryDeviationX = sqr(odometryTranslation.x() * odoDeviation.x());
  float odometryDeviationY = sqr(odometryTranslation.y() * odoDeviation.y());
  //process noise
  odometryNoiseX = sqr(pNp) + odometryDeviationX;
  odometryNoiseY = sqr(pNp) + odometryDeviationY;

  for(auto& obstacle : iWantToBeAnObstacle)
    obstacle.dynamic(odometryRotation, odometryTranslation, odometryJacobian, odometryNoiseX, odometryNoiseY, oKFDynamicNoise, dKFDynamicNoise);
}

void ObstacleModelProvider::deleteObstacles()
{
  for(auto obstacle = iWantToBeAnObstacle.begin(); obstacle != iWantToBeAnObstacle.end();)
  {
    const float obstacleRadius = obstacle->type == Obstacle::goalpost ? theFieldDimensions.goalPostRadius : Obstacle::getRobotDepth();
    const float centerDistanceSquared = obstacle->center.squaredNorm();
    if(obstacle->notSeenButShouldSeenCount >= notSeenThreshold
       || obstacle->lastSeen + deleteAfter <= theFrameInfo.time
       || centerDistanceSquared >= maxDistOnFieldSquared || centerDistanceSquared >= sqr(maxDistance)
       || (std::abs(centerDistanceSquared) <= sqr(obstacleRadius * .5f)) //obstacle is really inside us
       //|| obstacle->velocity.squaredNorm() > sqr(maxVelocity)  //velocity is deactivated
      )
    {
      obstacle = iWantToBeAnObstacle.erase(obstacle);
    }
    else
    {
      ++obstacle;
    }
  }
}

Matrix2f ObstacleModelProvider::getCovOfPointInWorld(const Vector2f& pointInWorld2) const
{
  Vector3f point3D;
  point3D << pointInWorld2, 0.f;
  const Vector3f unscaledVectorToPoint = theCameraMatrix.inverse() * point3D;
  const Vector3f unscaledWorld = theCameraMatrix.rotation * unscaledVectorToPoint;
  const float scale = theCameraMatrix.translation.z() / -unscaledWorld.z();
  const Vector2f pointInWorld(unscaledWorld.x() * scale, unscaledWorld.y() * scale);
  const float distance = pointInWorld.norm();
  Vector2f cossin;
  if(distance == 0.f)
  {
    cossin << 1.f, 0.f;
  }
  else
  {
    cossin = pointInWorld * (1.f / distance);
  }
  Matrix2f rot;
  rot << cossin.x(), -cossin.y(), cossin.y(), cossin.x();
  const Vector2f& robotRotationDeviation = theMotionInfo.isStanding() ? pRobotRotationDeviationInStand : pRobotRotationDeviation;
  Matrix2f cov;
  cov << sqr(theCameraMatrix.translation.z() / std::tan((distance == 0.f ? pi_2 : std::atan(theCameraMatrix.translation.z() / distance))
             - robotRotationDeviation.x()) - distance), 0.f,
             0.f, sqr(std::tan(robotRotationDeviation.y()) * distance);
  Matrix2f result = rot * cov * rot.transpose();
  Covariance::fixCovariance(result);
  return result;
}

void ObstacleModelProvider::propagateObstacles(ObstacleModel& obstacleModel) const
{
  obstacleModel.obstacles.clear();
  for(const auto& ob : iWantToBeAnObstacle)
  {
    if(ob.seenCount >= minPercepts || debug)
      obstacleModel.obstacles.emplace_back(ob);
  }
}

void ObstacleModelProvider::tryToMerge(const InternalObstacle& measurement)
{
  if(iWantToBeAnObstacle.empty())
  {
    iWantToBeAnObstacle.emplace_back(measurement);
    merged.push_back(true);
    return;
  }
  float distanceSquared = 0.f;
  const float thisMergeDistance = measurement.type == Obstacle::goalpost ? goalMergeDistance : mergeDistance;
  float possibleMergeDistSquared = maxDistOnFieldSquared;
  size_t atMerge = 0; //element matching the merge condition
  size_t noEKF = std::numeric_limits<size_t>::max(); //hopefully, this is not reached
  for(size_t i = 0; i < iWantToBeAnObstacle.size(); ++i)
  {
    if(merged[i])
      continue;
    const float mergeBonsu = (measurement.center.squaredNorm() / sqr(1000.f));
    const float robotDepths = (mergeBonsu >= 2.f && mergeBonsu <= 6.f ? std::floor(mergeBonsu) : 0.f) * Obstacle::getRobotDepth(); //for every meter there is a bonus of a robot radius in mm
    const float mergeDistanceSquared = sqr(robotDepths + thisMergeDistance);
    distanceSquared = (measurement.center - iWantToBeAnObstacle[i].center).squaredNorm();
    if(distanceSquared <= mergeDistanceSquared && distanceSquared <= possibleMergeDistSquared) //found probably matching obstacle
    {
      possibleMergeDistSquared = distanceSquared;
      atMerge = i;
    }
  }

  if(possibleMergeDistSquared < maxDistOnFieldSquared)
  {
    LINE("module:ObstacleModelProvider:merge", measurement.center.x(), measurement.center.y(),
         iWantToBeAnObstacle[atMerge].center.x(), iWantToBeAnObstacle[atMerge].center.y(), 10, Drawings::dashedPen, ColorRGBA::red);
    if(noEKF != atMerge)
    {
      iWantToBeAnObstacle[atMerge].measurement(measurement, weightedSum, theFieldDimensions, oKFMeasureNoise, dKFMeasureNoise); //EKF
    }
    iWantToBeAnObstacle[atMerge].considerType(measurement, colorThreshold, uprightThreshold);
    iWantToBeAnObstacle[atMerge].lastSeen = measurement.lastSeen;
    iWantToBeAnObstacle[atMerge].seenCount += measurement.seenCount;
    iWantToBeAnObstacle[atMerge].notSeenButShouldSeenCount = 0; //reset that counter
    merged[atMerge] = true;
    return;
  }
  //did not found possible match
  iWantToBeAnObstacle.emplace_back(measurement);
  merged.push_back(true);
}

void ObstacleModelProvider::setLeftRight()
{
  for(auto& obstacle : iWantToBeAnObstacle)
    obstacle.setLeftRight((obstacle.left - obstacle.right).norm() * .5f);
}

void ObstacleModelProvider::mergeOverlapping()
{
  if(iWantToBeAnObstacle.size() < 2)
    return;
  for(size_t i = 0; i < iWantToBeAnObstacle.size(); ++i)
  {
    for(size_t j = iWantToBeAnObstacle.size() - 1; j > i; --j)
    {
      //seen in this frame (obviously should not merged) and due to oscillating obstacles should also not merged
      if((iWantToBeAnObstacle[i].lastSeen + frameTimeDiff >= theFrameInfo.time && iWantToBeAnObstacle[j].lastSeen + frameTimeDiff >= theFrameInfo.time)
         || (std::max(iWantToBeAnObstacle[i].lastSeen, iWantToBeAnObstacle[j].lastSeen) - std::min(iWantToBeAnObstacle[i].lastSeen, iWantToBeAnObstacle[j].lastSeen) < mergeOverlapTimeDiff))
        continue;
      const float overlapSquared = ((iWantToBeAnObstacle[i].left - iWantToBeAnObstacle[i].right) * .5f).squaredNorm() + ((iWantToBeAnObstacle[j].left - iWantToBeAnObstacle[j].right) * .5f).squaredNorm();
      const float distanceOfCentersSquared = (iWantToBeAnObstacle[j].center - iWantToBeAnObstacle[i].center).squaredNorm();
      if(((distanceOfCentersSquared <= overlapSquared || distanceOfCentersSquared < sqr(2 * Obstacle::getRobotDepth())) &&
          ((iWantToBeAnObstacle[i].type >= Obstacle::unknown && iWantToBeAnObstacle[j].type >= Obstacle::unknown)
           || iWantToBeAnObstacle[i].type == iWantToBeAnObstacle[j].type)))
      {
        Obstacle::fusion2D(iWantToBeAnObstacle[i], iWantToBeAnObstacle[j]);
        if(iWantToBeAnObstacle[i].type == Obstacle::goalpost)
          iWantToBeAnObstacle[i].setLeftRight(theFieldDimensions.goalPostRadius);
        iWantToBeAnObstacle[i].considerType(iWantToBeAnObstacle[j], colorThreshold, uprightThreshold);
        iWantToBeAnObstacle[i].lastSeen = std::max(iWantToBeAnObstacle[i].lastSeen, iWantToBeAnObstacle[j].lastSeen);
        iWantToBeAnObstacle[i].seenCount = std::max(iWantToBeAnObstacle[i].seenCount, iWantToBeAnObstacle[j].seenCount);
        iWantToBeAnObstacle[i].notSeenButShouldSeenCount = iWantToBeAnObstacle[i].notSeenButShouldSeenCount + iWantToBeAnObstacle[j].notSeenButShouldSeenCount / 2;

        iWantToBeAnObstacle.erase(iWantToBeAnObstacle.begin() + j);
      }
    }
  }
}

void ObstacleModelProvider::considerTeammates()
{
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:teammates", "drawingOnField");
  if(!useTeammatePositionForClassification)
    return;

  //don't risk false classification if no communication is possibles
  if(iWantToBeAnObstacle.empty() || theTeamData.teammates.empty())
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
    InternalObstacle measurement(cov, relativePosition, Vector2f::Zero(), Vector2f::Zero(),
                                 teammate.timeWhenLastPacketReceived, minPercepts / 2,
                                 Obstacle::teammate);
    measurement.setLeftRight(Obstacle::getRobotDepth());
    //if the teammate is found in the local obstacle model, one would like to make sure, it is not classified as an opponent
    measurement.color = (colorThreshold / 2) * sgn(measurement.color);
    float distanceSquared = 0.f;
    float possibleMergeDistSquared = maxDistOnFieldSquared;
    size_t atMerge = 0; //element matching the merge condition
    for(size_t i = 0; i < iWantToBeAnObstacle.size(); ++i)
    {
      if(mergedTeammates.find(i) != mergedTeammates.end())
        continue;
      const float mergeBonsu = (measurement.center.squaredNorm() / sqr(1000.f));
      const float robotDepths = (mergeBonsu >= 2.f && mergeBonsu <= 6.f ? std::floor(mergeBonsu) : 0.f) * Obstacle::getRobotDepth(); //for every meter there is a bonus of a robot radius in mm
      const float mergeDistanceSquared = sqr(robotDepths + mergeDistance);
      distanceSquared = (measurement.center - iWantToBeAnObstacle[i].center).squaredNorm();
      if(distanceSquared <= mergeDistanceSquared && distanceSquared <= possibleMergeDistSquared) //found probably matching obstacle
      {
        possibleMergeDistSquared = distanceSquared;
        atMerge = i;
      }
    }
    if(possibleMergeDistSquared < maxDistOnFieldSquared)
    {
      iWantToBeAnObstacle[atMerge].considerType(measurement, colorThreshold, uprightThreshold);
      iWantToBeAnObstacle[atMerge].lastSeen = theFrameInfo.time;
      mergedTeammates.insert(atMerge);
    }
    else if(relativePosition.x() < -300.f || distanceSquared > sqr(maxDistance)) //magic number; teammate is behind me or too far away
      ++neverSeen;
  }
  //found all my teammates? other robots should be in the other team
  if(mergedTeammates.size() + neverSeen != theTeamData.teammates.size())
    return;
  InternalObstacle opponent(Obstacle::opponent);
  opponent.upright = 0;
  for(size_t i = 0; i < iWantToBeAnObstacle.size(); ++i)
  {
    if(mergedTeammates.find(i) != mergedTeammates.end())
    {
      mergedTeammates.erase(i);
      continue;
    }
    if(iWantToBeAnObstacle[i].type > Obstacle::unknown)
      iWantToBeAnObstacle[i].considerType(opponent, colorThreshold, uprightThreshold);
  }
}

void ObstacleModelProvider::shouldBeSeen()
{
  if(iWantToBeAnObstacle.empty())
    return;

  const bool useFieldBoundary = theFieldBoundary.isValid && !theFieldBoundary.boundarySpots.empty();
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

  for(size_t i = 0; i < iWantToBeAnObstacle.size(); ++i)
  {
    //check whether the obstacle could be seen in the image
    InternalObstacle* closer = &(iWantToBeAnObstacle[i]);
    Vector2f centerInImage;
    //check if obstacle is not in sight and was not seen for 300ms
    if(closer->lastSeen + 300u > theFrameInfo.time || !closer->isBetween(cameraAngleLeft, cameraAngleRight) ||
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
    if(useFieldBoundary &&
       iWantToBeAnObstacle[i].fieldBoundaryFurtherAsObstacle(centerInImage, notSeenThreshold, theCameraInfo, theCameraMatrix, theFieldBoundary))
      continue;
    //obstacle is not seen
    ++closer->notSeenButShouldSeenCount;
  }
}

bool ObstacleModelProvider::shadowedRobots(InternalObstacle* closer, const size_t i, const float cameraAngleLeft, const float cameraAngleRight)
{
  for(size_t j = iWantToBeAnObstacle.size() - 1; j > i; --j)
  {
    InternalObstacle* further = &(iWantToBeAnObstacle[j]);
    if(further->lastSeen + frameTimeDiff >= theFrameInfo.time)
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
