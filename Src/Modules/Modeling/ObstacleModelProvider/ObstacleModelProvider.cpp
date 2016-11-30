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
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/Obstacle.h"
#include <algorithm>
#include <limits>
#include <set>

MAKE_MODULE(ObstacleModelProvider, modeling)

void ObstacleModelProvider::update(ObstacleModel& obstacleModel)
{
  frameTimeDiff = static_cast<unsigned int>(theFrameInfo.cycleTime * 1000.f);
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:maxDistance", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:merge", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:cameraAngle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:obstacleNotSeen", "drawingOnImage");
  CIRCLE("module:ObstacleModelProvider:maxDistance", theRobotPose.translation.x(), theRobotPose.translation.y(), maxDistance, 6, Drawings::dottedPen,
         ColorRGBA::black, Drawings::noBrush, ColorRGBA::black);
  DEBUG_RESPONSE_ONCE("module:ObstacleModelProvider:clear")
  {
    iWantToBeAnObstacle.clear();
  }
  if(theRobotInfo.penalty != PENALTY_NONE
     || theGameInfo.state == STATE_INITIAL
     //while falling down / getting up / the obstacles might be invalid, better clean up
     || (theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::falling)
     || theMotionInfo.motion == MotionRequest::getUp
     || theGameInfo.secondaryState == STATE2_PENALTYSHOOT) //penalty shootout -> obstacles will be ignored
  {
    if(theGameInfo.state != STATE_FINISHED) //if the gamecontroller operator fails epicly and resets from finished to playing
    {
      obstacleModel.obstacles.clear();
      iWantToBeAnObstacle.clear();
    }
    return;
  }
  STOPWATCH("ObstacleModelProvider:delete") deleteObstacles(); //delete old obstacles and obstacles that are no longer obstacles for reasons
  STOPWATCH("ObstacleModelProvider:dynamic") dynamic(); //apply extended kalman filter prediction step to obstacles
  if(useArmContactModel) { STOPWATCH("ObstacleModelProvider:addArm") addArmContacts(); } //whether to us the arm contact model or not
  if(useFootBumperState) { STOPWATCH("ObstacleModelProvider:addFoot") addFootContacts(); } //whether to use the foot contact model or not
  STOPWATCH("ObstacleModelProvider:addPlayers") addPlayerPercepts(); //try to merge (if not, add new obstacle) with obstacles
  if(useTeammatePositionForClassification) { STOPWATCH("ObstacleModelProvider:considerTeammates") considerTeammates(); } //correct whether teammate or opponent player by given pose
  STOPWATCH("ObstacleModelProvider:mergeOverlapping") mergeOverlapping(); //overlapping obstacles are merged together
  STOPWATCH("ObstacleModelProvider:setLeftRight") setLeftRight(); //orthogonal obstacle.left/right
  STOPWATCH("ObstacleModelProvider:shouldBeSeen") shouldBeSeen(); //obstacles that are obviously not present will be deleted
  STOPWATCH("ObstacleModelProvider:propagate") propagateObstacles(obstacleModel); //put valid obstacles in representation
  BH_TRACE;
}

void ObstacleModelProvider::addArmContacts()
{
  if(!theArmContactModel.contactLeft && !theArmContactModel.contactRight)
    return;
  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());
  Vector2f center;
  //left arm
  if(theArmContactModel.contactLeft && theFrameInfo.getTimeSince(theArmContactModel.timeOfLastContactLeft) <= 2000)
  {
    if(!leftArmContact)
    {
      ANNOTATION("ObstacleModelProvider", "LeftArmContact.");
      leftArmContact = true;
    }
    center = (theTorsoMatrix.inverse() * theRobotModel.limbs[Limbs::shoulderLeft].translation).topRows(2);
    center.y() += Obstacle::getRobotDepth() + 15.f;
    InternalObstacle obstacle = InternalObstacle(armCov, center, Vector2f::Zero(), Vector2f::Zero(), theFrameInfo.time, 1,
                                                 Obstacle::unknown);
    obstacle.setLeftRight(Obstacle::getRobotDepth());
    //Insert valid obstacle
    tryToMerge(obstacle);
  }
  else
    leftArmContact = false;
  //right arm
  if(theArmContactModel.contactRight && theFrameInfo.getTimeSince(theArmContactModel.timeOfLastContactRight) <= 2000)
  {
    if(!rightArmContact)
    {
      ANNOTATION("ObstacleModelProvider", "RightArmContact.");
      rightArmContact = true;
    }
    center = (theTorsoMatrix.inverse() * theRobotModel.limbs[Limbs::shoulderRight].translation).topRows(2);
    center.y() -= Obstacle::getRobotDepth() + 15.f;
    InternalObstacle obstacle = InternalObstacle(armCov, center, Vector2f::Zero(), Vector2f::Zero(), theFrameInfo.time, 1,
                                                 Obstacle::unknown);
    obstacle.setLeftRight(Obstacle::getRobotDepth());
    //Insert valid obstacle
    tryToMerge(obstacle);
  }
  else
    rightArmContact = false;
}

void ObstacleModelProvider::addFootContacts()
{
  if(!theFootBumperState.contactLeft && !theFootBumperState.contactRight)
    return;
  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());
  Vector2f center;
  //left foot
  if(theFootBumperState.contactLeft && theFrameInfo.getTimeSince(theFootBumperState.lastContactLeft) <= 2000)
  {
    if(!leftFootContact)
    {
      ANNOTATION("ObstacleModelProvider", "LeftFootContact.");
      leftFootContact = true;
    }
    center = (theTorsoMatrix.inverse() * theRobotModel.limbs[Limbs::footLeft].translation).topRows(2);
    center.x() += Obstacle::getRobotDepth() + 65.f + 15.f; //65mm should be the distance from the joint to the toe
    //65mm is the distance between the bumper and the joint
    InternalObstacle obstacle = InternalObstacle(feetCov, center, Vector2f::Zero(), Vector2f::Zero(), theFrameInfo.time,
                                                 1, Obstacle::unknown);
    obstacle.setLeftRight(Obstacle::getRobotDepth());
    //Insert valid obstacle
    tryToMerge(obstacle);
  }
  else
    leftFootContact = false;
  //right foot
  if(theFootBumperState.contactRight && theFrameInfo.getTimeSince(theFootBumperState.lastContactRight) <= 2000)
  {
    if(!rightFootContact)
    {
      ANNOTATION("ObstacleModelProvider", "RightFootContact.");
      rightFootContact = true;
    }
    center = (theTorsoMatrix.inverse() * theRobotModel.limbs[Limbs::footRight].translation).topRows(2);
    center.x() += Obstacle::getRobotDepth() + 65.f + 15.f;//65mm should be the distance from the joint to the toe
    //65mm is the distance between the bumper and the joint
    InternalObstacle obstacle = InternalObstacle(feetCov, center, Vector2f::Zero(), Vector2f::Zero(), theFrameInfo.time,
                                                 1, Obstacle::unknown);
    obstacle.setLeftRight(Obstacle::getRobotDepth());
    //Insert valid obstacle
    tryToMerge(obstacle);
  }
  else
    rightFootContact = false;
}

void ObstacleModelProvider::addPlayerPercepts()
{
  if(thePlayersPercept.players.empty())
    return;
  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());
  for(const auto& percept : thePlayersPercept.players)
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
  {
    dynamic(obstacle);
  }
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
  float possibleMergeDistSquared = maxDistOnFieldSquared;
  size_t atMerge = 0; //element matching the merge condition
  size_t noEKF = std::numeric_limits<size_t>::max(); //hopefully, this is not reached
  for(size_t i = 0; i < iWantToBeAnObstacle.size(); ++i)
  {
    if(merged[i])
      continue;
    const float mergeBonsu = (measurement.center.squaredNorm() / sqr(1000.f));
    const float robotDepths = (mergeBonsu >= 2.f && mergeBonsu <= 6.f ? std::floor(mergeBonsu) : 0.f) * Obstacle::getRobotDepth(); //for every meter there is a bonus of a robot radius in mm
    const float mergeDistanceSquared = sqr(robotDepths + mergeDistance);
    distanceSquared = (measurement.center - iWantToBeAnObstacle[i].center).squaredNorm();
    if(distanceSquared <= mergeDistanceSquared //found probably matching obstacle
       && distanceSquared <= possibleMergeDistSquared)
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
      ObstacleModelProvider::measurement(iWantToBeAnObstacle[atMerge], measurement); //EKF
    }
    considerType(iWantToBeAnObstacle[atMerge], measurement);
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

void ObstacleModelProvider::dynamic(InternalObstacle& obstacle)
{
  //update the state
  obstacle.center.rotate(odometryRotation);
  obstacle.left.rotate(odometryRotation);
  obstacle.right.rotate(odometryRotation);

  obstacle.velocity.setZero();
  obstacle.center += odometryTranslation;
  obstacle.left += odometryTranslation;
  obstacle.right += odometryTranslation;

  //process new covariance matrix
  ASSERT(obstacle.covariance(0, 1) == obstacle.covariance(1, 0));
  obstacle.covariance = odometryJacobian * obstacle.covariance * odometryJacobian.transpose();
  obstacle.covariance(0, 0) += odometryNoiseX;
  obstacle.covariance(1, 1) += odometryNoiseY;
  Covariance::fixCovariance(obstacle.covariance);
}

void ObstacleModelProvider::measurement(InternalObstacle& obstacle, const InternalObstacle& measurement)
{
  //covariance matrices
  //computation of kalman gain, new state and covariance matrix
  ASSERT(obstacle.covariance(0, 1) == obstacle.covariance(1, 0));
  Matrix2f CXZ = obstacle.covariance;
  Matrix2f CZZ = measurement.covariance; //TODO: ...+CovZ
  CZZ.noalias() += CXZ;
  Matrix2f K = CXZ * CZZ.inverse();
  Vector2f muX;
  muX << obstacle.center;
  muX.noalias() += K * (measurement.center - obstacle.center);
  obstacle.covariance -= K * obstacle.covariance;

  obstacle.center << muX(0), muX(1);

  if(obstacle.type == Obstacle::goalpost)
  {
    obstacle.setLeftRight(theFieldDimensions.goalPostRadius);
  }
  else
  {
    float obstacleWidth = (obstacle.left - obstacle.right).norm();
    float measurementWidth = (measurement.left - measurement.right).norm();
    float width = (measurementWidth + obstacleWidth * (weightedSum - 1)) / weightedSum;
    if(width < 2.f * Obstacle::getRobotDepth())
      width = 2.f * Obstacle::getRobotDepth();
    obstacle.setLeftRight(width * .5f); //radius (that's why * .5f)
  }
  Covariance::fixCovariance(obstacle.covariance);
}

void ObstacleModelProvider::considerType(InternalObstacle& obstacle, const InternalObstacle& measurement)
{
  obstacle.color = std::min(colorThreshold * colorThreshold, std::max(measurement.color + obstacle.color, -colorThreshold * colorThreshold));
  obstacle.upright = std::min(2 * uprightThreshold, std::max(measurement.upright + obstacle.upright, -2 * uprightThreshold)); //'2' seems to be chosen wisely

  if(obstacle.type == measurement.type || measurement.type == Obstacle::unknown)
    return;
  if(obstacle.type == Obstacle::unknown)
  {
    obstacle.type = measurement.type;
    return;
  }
  if(obstacle.type == Obstacle::goalpost)
    return;
  if(measurement.type == Obstacle::goalpost)
  {
    obstacle.type = Obstacle::goalpost;
    return;
  }

  //the following code should perfectly consider whether a robot is fallen/upright and if it's an oppponent or teammate
  if(obstacle.color < colorThreshold && obstacle.color > -colorThreshold)
  {
    if(obstacle.upright <= -uprightThreshold)
    {
      obstacle.type = Obstacle::fallenSomeRobot;
      return;
    }
    obstacle.type = Obstacle::someRobot;
    return;
  }

  if(obstacle.color >= colorThreshold)
  {
    if(obstacle.upright <= -uprightThreshold)
    {
      obstacle.type = Obstacle::fallenOpponent;
      return;
    }
    obstacle.type = Obstacle::opponent;
    return;
  }
  else
  {
    if(obstacle.upright <= -uprightThreshold)
    {
      obstacle.type = Obstacle::fallenTeammate;
      return;
    }
    obstacle.type = Obstacle::teammate;
    return;
  }
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
           || iWantToBeAnObstacle[i].type == iWantToBeAnObstacle[j].type))) /*the following should eliminate obstacles overlapping with goalposts */
      {
        Obstacle::fusion2D(iWantToBeAnObstacle[i], iWantToBeAnObstacle[j]);
        considerType(iWantToBeAnObstacle[i], iWantToBeAnObstacle[j]);
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
  //don't risk false classification if no communication is possibles
  if(iWantToBeAnObstacle.empty() || theTeammateData.teammates.empty())
    return;

  size_t neverSeen = 0; //maybe never seen teammates

  std::set<size_t> mergedTeammates;
  for(const auto& teammate : theTeammateData.teammates)
  {
    if(teammate.status != Teammate::PLAYING)
      continue;

    const Matrix2f cov = teammate.pose.covariance.topLeftCorner(2, 2);
    ASSERT(cov(0, 1) == cov(1, 0));
    const Vector2f relativePosition = theRobotPose.inversePose * teammate.pose.translation;
    CIRCLE("module:ObstacleModelProvider:teammates", teammate.pose.translation.x(), teammate.pose.translation.y(), 100, 3, Drawings::dottedPen, ColorRGBA::black, Drawings::noBrush, ColorRGBA::yellow);
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
      considerType(iWantToBeAnObstacle[atMerge], measurement);
      iWantToBeAnObstacle[atMerge].lastSeen = theFrameInfo.time;
      mergedTeammates.insert(atMerge);
    }
    else if(relativePosition.x() < -300.f || distanceSquared > sqr(maxDistance)) //magic number; teammate is behind me or too far away
      ++neverSeen;
  }
  //found all my teammates? other robots should be in the other team
  if(mergedTeammates.size() + neverSeen != theTeammateData.teammates.size())
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
      considerType(iWantToBeAnObstacle[i], opponent);
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
    if(closer->lastSeen + 300u > theFrameInfo.time || !closer->isBetween(cameraAngleLeft, cameraAngleRight) || !isInImage(*closer, centerInImage))
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
    if(useFieldBoundary && fieldBoundaryFurtherAsObstacle(iWantToBeAnObstacle[i], centerInImage))
      continue;
    //obstacle is not seen
    ++closer->notSeenButShouldSeenCount;
  }
}

bool ObstacleModelProvider::isInImage(const InternalObstacle& obstacle, Vector2f& centerInImage)
{
  return Transformation::robotToImage(obstacle.center, theCameraMatrix, theCameraInfo, centerInImage)
         && centerInImage.x() < theCameraInfo.width - 10.f && centerInImage.x() > 10.f
         && centerInImage.y() < theCameraInfo.height - 10.f && centerInImage.y() > 10.f;
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
    if(!further->isBetween(cameraAngleLeft, cameraAngleRight) || !isInImage(*further, centerInImage2))
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

//use the boundary spots to make sure the absence of an obstacle (boundary spots are at the edge from green to garbage)
//the center of an obstacle is probably on a green scanline, so use the width of an obstacle and check if there are more
//points below the resulting line (obstacle left to right)
bool ObstacleModelProvider::fieldBoundaryFurtherAsObstacle(InternalObstacle& obstacle, const Vector2f& centerInImage)
{
  Vector2f leftInImage, rightInImage;
  const bool left = Transformation::robotToImage(obstacle.left, theCameraMatrix, theCameraInfo, leftInImage);
  const bool right = Transformation::robotToImage(obstacle.right, theCameraMatrix, theCameraInfo, rightInImage);
  int result = 0;
  for(const auto& boundarySpot : theFieldBoundary.boundarySpots)
  {
    //obstacle is behind the field boundary (obstacle could be seen but is not present)
    if(left && right && boundarySpot.x() >= leftInImage.x() && boundarySpot.x() <= rightInImage.x())
    {
      if(boundarySpot.y() < leftInImage.y() && boundarySpot.y() < rightInImage.y())
        --result;
      else
        ++result;
    }
  }
  if(result < -1)
  {
    obstacle.notSeenButShouldSeenCount += std::max(1u, notSeenThreshold / 10);
    return true;
  }

  return false;
}

InternalObstacle::InternalObstacle(const Matrix2f& covariance, const Vector2f& center, const Vector2f& left, const Vector2f& right,
                                   const unsigned lastSeen, const unsigned seenCount, const Type type)
{
  this->covariance << covariance;
  ASSERT(covariance(1, 0) == covariance(0, 1));
  this->center = center;
  this->left = left;
  this->right = right;
  this->velocity.setZero();
  this->type = type;
  this->lastSeen = lastSeen;
  this->seenCount = seenCount;
  notSeenButShouldSeenCount = 0u;
  upright = type < fallenSomeRobot ? 1 : -1;
  if(type == fallenTeammate || type == teammate)
    color = -1;
  else if(type == fallenOpponent || type == opponent)
    color = 1;
  else
    color = 0;
}

InternalObstacle::InternalObstacle(const Type type)
{
  upright = type < fallenSomeRobot ? 1 : -1;
  if(type == fallenTeammate || type == teammate)
    color = -1;
  else if(type == fallenOpponent || type == opponent)
    color = 1;
  else
    color = 0;
  this->type = type;
}

bool InternalObstacle::isBehind(const InternalObstacle& other) const
{
  const bool allPointsFurtherAway = left.squaredNorm() > other.left.squaredNorm() && right.squaredNorm() > other.right.squaredNorm();
  const bool centerBetweenLeftRight = left.angle() < other.center.angle() && right.angle() > other.center.angle();
  return centerBetweenLeftRight && allPointsFurtherAway;
}
