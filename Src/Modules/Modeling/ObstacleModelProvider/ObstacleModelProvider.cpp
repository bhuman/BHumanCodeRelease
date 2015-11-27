/**
 * @file ObstacleModelProvider.cpp
 *
 * This is the implementation of a module that combines arm contacts, foot bumper contacts and
 * obstacle percepts (from the module PlayersPerceptor) into one obstacle model.
 *
 * @author <a href="mailto:flomaass@informatik.uni-bremen.de">Florian Maaß</a>
 */

#include "ObstacleModelProvider.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/Obstacle.h"
#include <algorithm>
#include <limits>
#include <set>

MAKE_MODULE(ObstacleModelProvider, modeling)

void ObstacleModelProvider::update(ObstacleModel& obstacleModel)
{
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
  if(useArmContactModel){ STOPWATCH("ObstacleModelProvider:addArm") addArmContacts(); } //whether to us the arm contact model or not
  if(useFootContactModel){ STOPWATCH("ObstacleModelProvider:addFoot") addFootContacts(); } //whether to use the foot contact model or not
  if(useGoalPercepts) { STOPWATCH("ObstacleModelProvider:addGoal") addGoalPercepts(); } //whether to use goal percepts or not
  STOPWATCH("ObstacleModelProvider:addPlayers") addPlayerPercepts(); //try to merge (if not, add new obstacle) with obstacles
  STOPWATCH("ObstacleModelProvider:considerTeammates") considerTeammates(); //correct whether teammate or opponent player by given pose
  STOPWATCH("ObstacleModelProvider:mergeOverlapping") mergeOverlapping(); //overlapping obstacles are merged together
  STOPWATCH("ObstacleModelProvider:shouldBeSeen") shouldBeSeen(); //obstacles that are obviously not present will be deleted
  STOPWATCH("ObstacleModelProvider:propagate") propagateObstacles(obstacleModel); //put valid obstacles in representation
  BH_TRACE;
  if(obstacleModel.obstacles.size() > sizeOfObstacles)
  {
    ANNOTATION("ObstacleModelProvider", "over " << static_cast<unsigned>(sizeOfObstacles) << " obstacles in model!");
    sizeOfObstacles += 10;
  }
  if(iWantToBeAnObstacle.size() > sizeOfObstacles)
  {
    ANNOTATION("ObstacleModelProvider", "over " << static_cast<unsigned>(sizeOfObstacles) << " probable obstacles");
  }
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
  if(!theFootContactModel.contactLeft && !theFootContactModel.contactRight)
    return;
  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());
  Vector2f center;
  //left foot
  if(theFootContactModel.contactLeft && theFrameInfo.getTimeSince(theFootContactModel.lastContactLeft) <= 2000)
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
  if(theFootContactModel.contactRight && theFrameInfo.getTimeSince(theFootContactModel.lastContactRight) <= 2000)
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

    if(useGoalPercepts)
    {
      const Vector2f correctedImageCoordinates = theImageCoordinateSystem.toCorrected(Vector2i(percept.realCenterX, percept.y2));
      //ignore percepts that are (probably) goal posts
      bool drop = false;
      for(const auto& goalPost : theGoalPercept.goalPosts)
      {
        if(std::abs(goalPost.positionInImage.x() - static_cast<int>(correctedImageCoordinates.x())) <= playersPerceptGoalPerceptdiff &&
           goalPost.positionInImage.y() >= correctedImageCoordinates.y())
        {
          drop = true;
          break;
        }
      }
      if(drop)
        continue;
    }
    if(percept.centerOnField.squaredNorm() >= MAX_DIST_ON_FIELD2 || percept.centerOnField.squaredNorm() >= sqr(maxDistance))
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
    else if(percept.detectedJersey && !ignoreJersey) //ignoreJersey HACK, please remove after RoboCup/
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
  for(auto& obstacle : iWantToBeAnObstacle)
  {
    dynamic(obstacle);
  }
}

void ObstacleModelProvider::deleteObstacles()
{
  if(iWantToBeAnObstacle.empty())
    return;
  if(useGoalPercepts)
  {
    //Sort out some goals
    for(size_t i = 0; i < iWantToBeAnObstacle.size(); ++i)
    {
      for(size_t j = iWantToBeAnObstacle.size() - 1; j > i; --j)
      {
        if(iWantToBeAnObstacle[i].type == Obstacle::goalpost &&
           iWantToBeAnObstacle[j].type == Obstacle::goalpost)
        {
          //now make sure, there are only four goal posts
          //closest goal percept to an goal post should be the best candidate to merge
          //big distances between goals indicate to different goals
          //merge somewhat plausible goal posts
          const float distance = (iWantToBeAnObstacle[i].center - iWantToBeAnObstacle[j].center).squaredNorm();
          const float angle = std::abs(Angle::normalize(iWantToBeAnObstacle[i].center.angle() - iWantToBeAnObstacle[j].center.angle())); //should be less than maxAngle
          if(distance <= maxGoalToGoalDistSqr && angle < maxGoalAngle && iWantToBeAnObstacle[i].seenCount >= iWantToBeAnObstacle[j].seenCount)
          {
            Obstacle::fusion2D(iWantToBeAnObstacle[i], iWantToBeAnObstacle[j]);
            iWantToBeAnObstacle[i].setLeftRight(theFieldDimensions.goalPostRadius);
            considerType(iWantToBeAnObstacle[i], iWantToBeAnObstacle[j]);
            iWantToBeAnObstacle.erase(iWantToBeAnObstacle.begin() + j);
          }
          //else if(squaredDistance > lowerDistOtherGoalSqr)// && distance < upperDistOtherGoal)
          //{
          //  continue; //probably a different goal post
          //}
          //else
          //{
          //  //OUTPUT_TEXT("dist: " << distance << " angle: " << angle); //debugoutput
          //  iWantToBeAnObstacle.erase(iWantToBeAnObstacle.begin() + j);
          //}
          /*
          else if(distance > maxGoalToGoalDist && distance <= theFieldDimensions.xPosOpponentGroundline && angle < maxAngle && obstacle->seenCount >= otherObstacle->seenCount)
          {
          otherObstacle = iWantToBeAnObstacle.erase(otherObstacle);
          if(otherObstacle < obstacle)
          --obstacle;
          }*/
        }
      }
    }
  }
  for(auto obstacle = iWantToBeAnObstacle.begin(); obstacle != iWantToBeAnObstacle.end();)
  {
    //ASSERT(obstacle->seenCount <= maxPercepts);
    //Decrease seencount once per second (if lastMeasurement is one second old)
    //if(theCameraInfo.camera && obstacle->seenCount >= 1 && (obstacle->lastMeasurement + 1000) <= theFrameInfo.time)
    //{
    //  --obstacle->seenCount;
    //  obstacle->lastMeasurement = theFrameInfo.time;
    //}
    const float centerDistance = obstacle->center.squaredNorm();
    if(obstacle->notSeenButShouldSeenCount >= notSeenThreshold ||
       obstacle->lastMeasurement + deleteAfter <= theFrameInfo.time ||
       centerDistance >= MAX_DIST_ON_FIELD2 || centerDistance >= sqr(maxDistance) ||
       (std::abs(centerDistance) <= sqr(Obstacle::getRobotDepth() / 2.f)) || //obstacle is really inside us
       obstacle->velocity.squaredNorm() > sqr(maxVelocity)
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

void ObstacleModelProvider::addGoalPercepts()
{
  if(theGoalPercept.goalPosts.empty())
    return;
  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());
  for(const auto& goalPost : theGoalPercept.goalPosts)
  {
    if(goalPost.positionOnField.squaredNorm() >= MAX_DIST_ON_FIELD2 || goalPost.positionOnField.squaredNorm() >= sqr(maxDistance))
      continue;
    unsigned confidence = 1;
    if(theGoalPercept.timeWhenCompleteGoalLastSeen == theFrameInfo.time)
    {
      confidence = minPercepts;
    }
    InternalObstacle obstacle = InternalObstacle(getCovOfPointInWorld(goalPost.positionOnField), goalPost.positionOnField, Vector2f::Zero(),
                                                 Vector2f::Zero(), theFrameInfo.time, confidence, Obstacle::goalpost);
    obstacle.setLeftRight(theFieldDimensions.goalPostRadius);
    tryToMerge(obstacle);
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
    ANNOTATION("ObstacleModelProvider", "distance is zero");
    cossin << 1.f, 0.f;
  }
  else
  {
    cossin = pointInWorld * (1.f / distance);
  }
  Matrix2f rot;
  rot << cossin.x(), -cossin.y(), cossin.y(), cossin.x();
  const Vector2f& robotRotationDeviation = (theMotionInfo.motion == MotionRequest::stand ||
      (theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standHigh))
      ? pRobotRotationDeviationInStand : pRobotRotationDeviation;
  Matrix2f cov;
  cov << sqr(theCameraMatrix.translation.z() / std::tan((distance == 0.f ? pi_2 : std::atan(theCameraMatrix.translation.z() / distance))
             - robotRotationDeviation.x()) - distance), 0.f,
             0.f, sqr(std::tan(robotRotationDeviation.y()) * distance);
  const Matrix2f result = rot * cov * rot.transpose();
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
  float distance = 0.f;
  const float thisMergeDistance = measurement.type == Obstacle::goalpost ? goalMergeDistanceSqr : mergeDistanceSqr;
  float possibleMergeDist = MAX_DIST_ON_FIELD2;
  size_t atMerge = 0; //element matching the merge condition
  size_t noEKF = std::numeric_limits<size_t>::max(); //hopefully, this is not reached
  for(size_t i = 0; i < iWantToBeAnObstacle.size(); ++i)
  {
    if(merged[i])
      continue;
    const float mergeBonsu = 0;// measurement.center.norm() / 1000.f * Obstacle::getRobotDepth(); //for every meter there is a bonus of one robot depth (due to noise)
    distance = (measurement.center - iWantToBeAnObstacle[i].center).squaredNorm();
    if(distance <= thisMergeDistance + sqr(mergeBonsu)) //found probably matching obstacle
    {
      if(useGoalPercepts && iWantToBeAnObstacle[i].type == Obstacle::goalpost && measurement.type != Obstacle::goalpost)
      {
        if(distance >= minimalDistancetoGoalpostSqr) //measured (maybe) a robot
          continue;

        //no EKF, old goalpercepts are more trustworthy.
        //maybe the goal frame was measured
        noEKF = i;
      }
      if(distance <= possibleMergeDist)
      {
        possibleMergeDist = distance;
        atMerge = i;
      }
    }
  }

  if(possibleMergeDist < MAX_DIST_ON_FIELD2)
  {
    if(noEKF != atMerge)
    {
      ObstacleModelProvider::measurement(iWantToBeAnObstacle[atMerge], measurement); //EKF
    }
    considerType(iWantToBeAnObstacle[atMerge], measurement);
    iWantToBeAnObstacle[atMerge].seenCount += measurement.seenCount;
    iWantToBeAnObstacle[atMerge].notSeenButShouldSeenCount -= iWantToBeAnObstacle[atMerge].notSeenButShouldSeenCount > 0 ? 1 : 0;
    merged[atMerge] = true;
    return;
  }
  //did not found possible match
  iWantToBeAnObstacle.emplace_back(measurement);
  merged.push_back(true);
}

void ObstacleModelProvider::dynamic(InternalObstacle& obstacle)
{
  float thisPNv2 = sqr(pNv);
  //update the state
  const float rot = -theOdometer.odometryOffset.rotation; //obstacle has to move in the opposite direction
  const Vector2f translation = theOdometer.odometryOffset.translation * -1.f;
  obstacle.center.rotate(rot);
  obstacle.left.rotate(rot);
  obstacle.right.rotate(rot);
  //create jacobian
  Matrix2f jacobian;
  /*jacobian << std::cos(rot), -std::sin(rot), theFrameInfo.cycleTime, 0.f,
              std::sin(rot), std::cos(rot), 0.f, theFrameInfo.cycleTime,
              0.f, 0.f, std::cos(rot), -std::sin(rot),
              0.f, 0.f, std::sin(rot), std::cos(rot);*/
  jacobian << std::cos(rot), -std::sin(rot), std::sin(rot), std::cos(rot);
  //if(obstacle.type == Obstacle::goalpost || obstacle.type == Obstacle::unknown)
  {
    obstacle.velocity.setZero();
    obstacle.center += translation;
    obstacle.left += translation;
    obstacle.right += translation;
    thisPNv2 = 0.f;
  }
  /*else
  {
  obstacle.velocity.setZero();// rotate(rot);
  obstacle.center += translation + obstacle.velocity * theFrameInfo.cycleTime;
  obstacle.left += translation + obstacle.velocity * theFrameInfo.cycleTime;
  obstacle.right += translation + obstacle.velocity * theFrameInfo.cycleTime;
  }*/
  //noise
  //todo add noise from rotation
  const float odometryDeviationX = sqr(translation.x() * odoDeviation.x());
  const float odometryDeviationY = sqr(translation.y() * odoDeviation.y());
  Matrix2f noise;
  noise << sqr(pNp) + odometryDeviationX, 0.f,// 0.f, 0.f,
    0.f, sqr(pNp) + odometryDeviationY;// , 0.f, 0.f,
  //0.f, 0.f, thisPNv2 + odometryDeviationX, 0.f,
  //0.f, 0.f, 0.f, thisPNv2 + odometryDeviationY;
  //process new covariance matrix
  obstacle.covariance = jacobian * obstacle.covariance * jacobian.transpose() + noise;
}

void ObstacleModelProvider::measurement(InternalObstacle& obstacle, const InternalObstacle& measurement)
{
  //jacobian matrix
  Matrix2f jacobian;
  jacobian << 1.f, 0.f,// 0.f, 0.f,
           0.f, 1.f;// , 0.f, 0.f;

  //coariance matrices
  //computation of kalman gain, new state and covariance matrix
  //here: unreadable but eigen efficient
  Matrix2f CXZ = obstacle.covariance * jacobian.transpose();
  Matrix2f CZZ = measurement.covariance; //...+CovZ
  CZZ.noalias() += jacobian * CXZ;
  Matrix2f K = CXZ * CZZ.inverse();
  Vector2f muX;
  muX << obstacle.center;// , obstacle.velocity;
  muX.noalias() += K * (measurement.center - obstacle.center);
  obstacle.covariance -= K * jacobian * obstacle.covariance;

  obstacle.center << muX(0), muX(1);

  if(obstacle.type == Obstacle::goalpost)
    obstacle.setLeftRight(theFieldDimensions.goalPostRadius);
  else
  {
    obstacle.left = measurement.left;
    obstacle.right = measurement.right;
  }

  //  if(obstacle.type != Obstacle::goalpost && obstacle.type != Obstacle::unknown)
  //    obstacle.velocity;// << muX(2), muX(3);
}

void ObstacleModelProvider::considerType(InternalObstacle& obstacle, const InternalObstacle& measurement)
{
  obstacle.lastMeasurement = std::max(obstacle.lastMeasurement, measurement.lastMeasurement);
  obstacle.seenCount = std::max(measurement.seenCount, obstacle.seenCount);
  obstacle.notSeenButShouldSeenCount = std::max(measurement.notSeenButShouldSeenCount, obstacle.notSeenButShouldSeenCount);
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

void ObstacleModelProvider::mergeOverlapping()
{
  if(iWantToBeAnObstacle.size() < 2)
    return;
  //const float overlap = sqr(2 * Obstacle::getRobotDepth());
  for(size_t i = 0; i < iWantToBeAnObstacle.size(); ++i)
  {
    for(size_t j = iWantToBeAnObstacle.size() - 1; j > i; --j)
    {
      //seen in this frame (obviously should not merged) and due to oscillating obstacles should also not merged
      if((iWantToBeAnObstacle[i].lastMeasurement == theFrameInfo.time && iWantToBeAnObstacle[j].lastMeasurement == theFrameInfo.time)
         || (std::max(iWantToBeAnObstacle[i].lastMeasurement, iWantToBeAnObstacle[j].lastMeasurement) - std::min(iWantToBeAnObstacle[i].lastMeasurement, iWantToBeAnObstacle[j].lastMeasurement) < mergeOverlapTimeDiff))
         continue;
      const float overlap = ((iWantToBeAnObstacle[i].left - iWantToBeAnObstacle[i].right) / 2.f).squaredNorm() + ((iWantToBeAnObstacle[j].left - iWantToBeAnObstacle[j].right) / 2.f).squaredNorm();
      if(((iWantToBeAnObstacle[j].center - iWantToBeAnObstacle[i].center).squaredNorm() <= overlap &&
          ((iWantToBeAnObstacle[i].type >= Obstacle::unknown && iWantToBeAnObstacle[j].type >= Obstacle::unknown)
           || iWantToBeAnObstacle[i].type == iWantToBeAnObstacle[j].type)) /*the following should eliminate obstacles overlapping with goalposts */
         || (useGoalPercepts &&
            (iWantToBeAnObstacle[j].center - iWantToBeAnObstacle[i].center).squaredNorm() <= minimalDistancetoGoalpostSqr &&
            (iWantToBeAnObstacle[i].type >= Obstacle::goalpost && iWantToBeAnObstacle[j].type >= Obstacle::goalpost)))
      {
        //fusion4D(iWantToBeAnObstacle[i], iWantToBeAnObstacle[j]);
        Obstacle::fusion2D(iWantToBeAnObstacle[i], iWantToBeAnObstacle[j]);
        if(iWantToBeAnObstacle[i].type == Obstacle::goalpost)
          iWantToBeAnObstacle[i].setLeftRight(theFieldDimensions.goalPostRadius);
        considerType(iWantToBeAnObstacle[i], iWantToBeAnObstacle[j]);
        iWantToBeAnObstacle.erase(iWantToBeAnObstacle.begin() + j);
      }
    }
  }
}

void ObstacleModelProvider::considerTeammates()
{
  DECLARE_DEBUG_DRAWING("module:ObstacleModelProvider:Teammates", "drawingOnField");
  //don't risk false classification if no communication is possibles
  if(iWantToBeAnObstacle.empty() || theTeammateData.teammates.empty())
    return;

  size_t neverSeen = 0; //maybe never seen teammates

  std::set<size_t> mergedTeammates;
  for(const auto& teammate : theTeammateData.teammates)
  {
    if(teammate.status != Teammate::FULLY_ACTIVE)
      continue;

    const Matrix2f cov = teammate.pose.covariance.topLeftCorner(2, 2);
    const Vector2f relativePosition = theRobotPose.inverse() * teammate.pose.translation;
    CIRCLE("module:ObstacleModelProvider:Teammates", relativePosition.x(), relativePosition.y(), 100, 3, Drawings::dottedPen, ColorRGBA::black, Drawings::noBrush, ColorRGBA::yellow);
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
    float distance = 0.f;
    float possibleMergeDist = MAX_DIST_ON_FIELD2;
    size_t atMerge = 0; //element matching the merge condition
    for(size_t i = 0; i < iWantToBeAnObstacle.size(); ++i)
    {
      if(mergedTeammates.find(i) != mergedTeammates.end())
        continue;
      /*if(iWantToBeAnObstacle[i].type < Obstacle::unknown)
        continue;*/
      const float mergeBonsu = (measurement.center.squaredNorm() / sqr(1000.f)) * Obstacle::getRobotDepth(); //for every meter there is a bonus of robot radius in mm
      distance = (measurement.center - iWantToBeAnObstacle[i].center).squaredNorm();
      if(distance <= mergeDistanceSqr + mergeBonsu && distance <= possibleMergeDist) //found probably matching obstacle
      {
        possibleMergeDist = distance;
        atMerge = i;
      }
    }
    if(possibleMergeDist < MAX_DIST_ON_FIELD2)
    {
      considerType(iWantToBeAnObstacle[atMerge], measurement);
      mergedTeammates.insert(atMerge);
    }
    else if(relativePosition.x() < -300.f || distance > maxDistance) //magic number; teammate is behind me or too far away
      ++neverSeen;
  }
  //found all my teammates? other robots should be in the other team
  if(mergedTeammates.size() + neverSeen != theTeammateData.teammates.size())
    return;
  InternalObstacle opponent(Obstacle::opponent);
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
  const float cameraAngleLeft = cameraAngle + theCameraInfo.openingAngleWidth * .45f,
    cameraAngleRight = cameraAngle - theCameraInfo.openingAngleWidth * .45f;
  for(size_t i = 0; i < iWantToBeAnObstacle.size(); ++i)
  {
    //check whether the obstacle could be seen in the image
    InternalObstacle* closer = &(iWantToBeAnObstacle[i]);
    Vector2f centerInImage;
    //check if obstacle is not in sight and was not seen for 300ms
    if(closer->lastMeasurement + 300u > theFrameInfo.time || !closer->isBetween(cameraAngleLeft, cameraAngleRight) || !isInImage(*closer, centerInImage))
       continue;

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

/*void ObstacleModelProvider::fusion4D(InternalObstacle& one, const InternalObstacle& other)
{
//multidimensional square equation (german: "Multidimensionale quadratische Ausgleichsrechnung" aus dem Skript
//"Theorie der Sensorfusion")
const Eigen::Matrix<float, 8, 4> A = (Eigen::Matrix<float, 8, 4>() << Matrix4f::Identity(), Matrix4f::Identity()).finished();
const Eigen::Matrix<float, 4, 8> AT = A.transpose();
const Eigen::Matrix<float, 8, 8> Sigma = ((Eigen::Matrix<float, 8, 8>() << one.covariance, Matrix4f::Zero(), Matrix4f::Zero(), other.covariance
).finished());
if(Sigma.determinant() == 0)
{
ANNOTATION("ObstacleModelProvider", "4dfusion determinant");
OUTPUT_ERROR("Call Florian: one is " << one.type << " has cov: " << one.covariance(0, 0) << " " << one.covariance(1, 1) << " " << one.covariance(2, 2) << " " << one.covariance(3, 3)
<< " other's type " << other.type << " has cov: " << other.covariance(0, 0) << " " << other.covariance(1, 1) << " " << other.covariance(2, 2) << " " << other.covariance(3, 3));
return;
}
const Eigen::Matrix<float, 8, 8> SigmaInv = Sigma.inverse();
const Eigen::Matrix<float, 8, 1> z = (Eigen::Matrix<float, 8, 1>() << one.center, one.velocity,
other.center, other.velocity).finished();
const Eigen::Matrix4f ATSigmaInvAINV = ((AT * SigmaInv) * A).inverse();
const Vector4f X = ATSigmaInvAINV * AT * SigmaInv * z;
ASSERT(X.allFinite());
ASSERT(ATSigmaInvAINV.allFinite());
one.covariance = ATSigmaInvAINV;
one.center << X(0), X(1);

if(one.type == Obstacle::goalpost)
one.setLeftRight(theFieldDimensions.goalPostRadius);
else
one.setLeftRight(Obstacle::getRobotDepth());

//if(one.type != Obstacle::goalpost && one.type != Obstacle::unknown)
//{
//  //one.velocity << X(2), X(3);
//}
}*/

bool ObstacleModelProvider::isInImage(const InternalObstacle& obstacle, Vector2f centerInImage) const
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
    if(further->lastMeasurement == theFrameInfo.time)
      continue;
    //check if obstacle is not in sight
    Vector2f centerInImage2;
    if(!further->isBetween(cameraAngleLeft, cameraAngleRight) || !isInImage(*further, centerInImage2))
      continue;
    //which obstacle is closer
    InternalObstacle::closerFurther(closer, further);
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

bool ObstacleModelProvider::fieldBoundaryFurtherAsObstacle(InternalObstacle& obstacle, const Vector2f centerInImage)
{
  Vector2f leftInImage, rightInImage;
  const bool left = Transformation::robotToImage(obstacle.left, theCameraMatrix, theCameraInfo, leftInImage);
  const bool right = Transformation::robotToImage(obstacle.right, theCameraMatrix, theCameraInfo, rightInImage);
  for(const auto& boundarySpot : theFieldBoundary.boundarySpots)
  {
    //obstacle is behind the field boundary (obstacle could be seen but is not present)
    if(left && right && boundarySpot.x() >= leftInImage.x() && boundarySpot.x() <= rightInImage.x()
       && boundarySpot.y() < leftInImage.y() && boundarySpot.y() < rightInImage.y())
    {
      obstacle.notSeenButShouldSeenCount += std::max(1u, notSeenThreshold / 10);
      return true;
    }
    else if(left && boundarySpot.x() >= leftInImage.x() && boundarySpot.x() <= centerInImage.x() &&
            boundarySpot.y() < centerInImage.y() && boundarySpot.y() < leftInImage.y())
    {
      obstacle.notSeenButShouldSeenCount += std::max(1u, notSeenThreshold / 10);
      return true;
    }
    else if(right && boundarySpot.x() >= centerInImage.x() && boundarySpot.x() <= rightInImage.x() &&
            boundarySpot.y() < centerInImage.y() && boundarySpot.y() < rightInImage.y())
    {
      obstacle.notSeenButShouldSeenCount += std::max(1u, notSeenThreshold / 10);
      return true;
    }
  }
  return false;
}

InternalObstacle::InternalObstacle(const Matrix2f& covariance, const Vector2f& center, const Vector2f& left, const Vector2f& right,
                                   const unsigned lastMeasurement, const unsigned seenCount, const Type type)
{
  this->covariance << covariance;// , Matrix2f::Zero(), Matrix2f::Zero(), (Matrix2f() << 10000.f, 0.f, 0.f, 10000.f).finished();
  this->center = center;
  this->left = left;
  this->right = right;
  this->velocity.setZero();
  this->type = type;
  this->lastMeasurement = lastMeasurement;
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
