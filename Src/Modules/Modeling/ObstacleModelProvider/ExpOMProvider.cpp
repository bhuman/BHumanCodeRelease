/**
 * @file ExpOMProvider.cpp
 *
 * This is the implementation of a module that combines arm contacts, foot bumper contacts,
 * obstacle percepts (from the module RobotPerceptor) and ultra sonic obstacles into one obstacle model.
 *
 * @author Florin
 */

#include "ExpOMProvider.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>
#include <Eigen/Dense>

MAKE_MODULE(ExpOMProvider, Modeling)

#define degreeOfFreedom 4
const float MAX_DIST_ON_FIELD2 = 10400.f * 10400.f + 7400.f * 7400.f;

void ExpOMProvider::update(ExpObstacleModel& expObstacleModel)
{
  DEBUG_RESPONSE_ONCE("module:ExpOMProvider:clear",
  {
    iWantToBeAnObstacle.clear();
  });
  if(theRobotInfo.penalty != PENALTY_NONE
     || (theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::falling)
     || (theGameInfo.state == STATE_FINISHED || theGameInfo.state == STATE_INITIAL)
     || theMotionInfo.motion == MotionRequest::getUp)
  {
    expObstacleModel.eobs.clear();
    iWantToBeAnObstacle.clear();
    lastFrame = theFrameInfo.time;
    return;
  }
  maxPercepts = minPercepts + deleteValue;
  deltaTime = static_cast<float>(theFrameInfo.time - lastFrame);
  STOP_TIME_ON_REQUEST("ExpOMProvider:delete", deleteObstacles(););
  STOP_TIME_ON_REQUEST("ExpOMProvider:dynamic", dynamic(););
  STOP_TIME_ON_REQUEST("ExpOMProvider:addArm", addArmContacts(););
  STOP_TIME_ON_REQUEST("ExpOMProvider:addFoot", addFootContacts(););
  STOP_TIME_ON_REQUEST("ExpOMProvider:addUS", addUSObstacles(););
  STOP_TIME_ON_REQUEST("ExpOMProvider:addGoal", addGoalPercepts(););
  STOP_TIME_ON_REQUEST("ExpOMProvider:addRobots", addRobotPercepts(););
  STOP_TIME_ON_REQUEST("ExpOMProvider:mergeOverlapping", mergeOverlapping(););
  STOP_TIME_ON_REQUEST("ExpOMProvider:propagate", propagateObstacles(expObstacleModel);); //put valid obstacles in representation
  lastFrame = theFrameInfo.time;
  DEBUG_RESPONSE_ONCE("module:ExpOMProvider:sortOutGoalsDebug",
  {
    OUTPUT_TEXT("minDistance: " << minDistance << " maxDistance: " << maxDistance << " minAngle: " << minAngle << " maxAngle: " << maxAngle);
  });
}

void ExpOMProvider::addArmContacts()
{
  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());
  //left arm
  if(theArmContactModel.contactLeft && theFrameInfo.getTimeSince(theArmContactModel.timeOfLastContactLeft) == 0)
  {
    const Vector2<> center(((theTorsoMatrix.invert() * theRobotModel.limbs[MassCalibration::shoulderLeft].translation).x),
                           ((theTorsoMatrix.invert() * theRobotModel.limbs[MassCalibration::shoulderLeft].translation).y));
    tryToMerge(InternalExpObstacle(armCov, center, theFrameInfo.time, maxPercepts, unknownVarianceVelocity));//Insert valid obstacle
  }
  //right arm
  if(theArmContactModel.contactRight && theFrameInfo.getTimeSince(theArmContactModel.timeOfLastContactRight) == 0)
  {
    const Vector2<> center(((theTorsoMatrix.invert() * theRobotModel.limbs[MassCalibration::shoulderRight].translation).x),
                           ((theTorsoMatrix.invert() * theRobotModel.limbs[MassCalibration::shoulderRight].translation).y));
    tryToMerge(InternalExpObstacle(armCov, center, theFrameInfo.time, maxPercepts, unknownVarianceVelocity));//Insert valid obstacle
  }
}

void ExpOMProvider::addFootContacts()
{
  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());
  //left foot
  if(theFootContactModel.contactLeft && theFrameInfo.getTimeSince(theFootContactModel.lastContactLeft) == 0)
  {
    const Vector2<> center(((theTorsoMatrix.invert() * theRobotModel.limbs[MassCalibration::footLeft].translation).x) + 110.f,
                           ((theTorsoMatrix.invert() * theRobotModel.limbs[MassCalibration::footLeft].translation).y));
    tryToMerge(InternalExpObstacle(feetCov, center, theFrameInfo.time, maxPercepts, unknownVarianceVelocity));//Insert valid obstacle
  }
  //right foot
  if(theFootContactModel.contactRight && theFrameInfo.getTimeSince(theFootContactModel.lastContactRight) == 0)
  {
    const Vector2<> center(((theTorsoMatrix.invert() * theRobotModel.limbs[MassCalibration::footRight].translation).x) + 110.f,
                           ((theTorsoMatrix.invert() * theRobotModel.limbs[MassCalibration::footRight].translation).y));
    tryToMerge(InternalExpObstacle(feetCov, center, theFrameInfo.time, maxPercepts, unknownVarianceVelocity));//Insert valid obstacle
  }
}

void ExpOMProvider::addUSObstacles()
{
  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());
  for(const auto& obstacle : theUSObstacleModel.obstacles)
  {
    tryToMerge(InternalExpObstacle(obstacle.covariance, obstacle.center, InternalExpObstacle::US, theFrameInfo.time, 1, unknownVarianceVelocity));
  }
}

void ExpOMProvider::addRobotPercepts()
{
  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());
  for(const auto& percept : theRobotPercept.robots)
  {
    if(!percept.detectedFeet)
      continue;

    Vector2<> center;
    const Vector2<> correctedImageCoordinates = theImageCoordinateSystem.toCorrected(Vector2<int>(percept.realCenterX, percept.y2));
    if(!Transformation::imageToRobot(correctedImageCoordinates.x, correctedImageCoordinates.y, theCameraMatrix, theCameraInfo, center))
      continue;
    center.normalize(center.abs() + robotDepth);
    if(center.squareAbs() >= MAX_DIST_ON_FIELD2)
      continue;
    const Matrix2x2<> cov = getCovOfPointInWorld(center);
    InternalExpObstacle::Type t = InternalExpObstacle::SOMEROBOT;

    if(percept.detectedJersey && percept.teamRed)
    {
      if(percept.fallen)
      {
        t = InternalExpObstacle::FALLENRED;
      }
      else
      {
        t = InternalExpObstacle::ROBOTRED;
      }
    }
    else if(percept.detectedJersey)
    {
      if(percept.fallen)
      {
        t = InternalExpObstacle::FALLENBLUE;
      }
      else
      {
        t = InternalExpObstacle::ROBOTBLUE;
      }
    }
    else if(percept.fallen)
    {
      t = InternalExpObstacle::SOMEFALLENROBOT;
    }

    tryToMerge(InternalExpObstacle(cov, center, t, theFrameInfo.time, 1, varianceVel));
  }
}

void ExpOMProvider::dynamic()
{
  for(auto& obstacle : iWantToBeAnObstacle)
  {
    dynamic(obstacle);
  }
}

void ExpOMProvider::deleteObstacles()
{
  if(iWantToBeAnObstacle.empty())
    return;
  //Sort out some goals
  for(int i = 0; i < static_cast<int>(iWantToBeAnObstacle.size()); ++i)
  {
    for(int j = static_cast<int>(iWantToBeAnObstacle.size()) - 1; j > i; --j)
    {
      if(iWantToBeAnObstacle[i].type == InternalExpObstacle::GOALPOST && iWantToBeAnObstacle[j].type == InternalExpObstacle::GOALPOST)
      {
        //now make sure, there are only four goal posts
        //closest goal percept to an goal post should be the best candidate to merge
        //big distances between goals indicate to different goals
        //merge somewhat plausible goal posts
        const float distance = (iWantToBeAnObstacle[i].center - iWantToBeAnObstacle[j].center).squareAbs();
        const float angle = std::abs(normalize(iWantToBeAnObstacle[i].center.angle() - iWantToBeAnObstacle[j].center.angle())); //should be less than maxAngle
        if(distance < 4000.f)//debugoutput
        {
          minDistance = std::min(minDistance, distance);
          maxDistance = std::max(maxDistance, distance);
          minAngle = std::min(minAngle, angle);
          maxAngle = std::max(maxAngle, angle);
        }
        if(distance <= maxGoalToGoalDistSqr && angle < maxGoalAngle && iWantToBeAnObstacle[i].seenCount >= iWantToBeAnObstacle[j].seenCount)
        {
          measurement(iWantToBeAnObstacle[i], iWantToBeAnObstacle[j]); //EKF

          iWantToBeAnObstacle[i].seenCount = std::min(iWantToBeAnObstacle[i].seenCount + iWantToBeAnObstacle[j].seenCount, maxPercepts);

          iWantToBeAnObstacle.erase(iWantToBeAnObstacle.begin() + j);
        }
        else if(distance > lowerDistOtherGoalSqr)// && distance < upperDistOtherGoal)
        {
          continue; //probably a different goal post
        }
        else
        {
          //OUTPUT_TEXT("dist: " << distance << " angle: " << angle); //debugoutput
          iWantToBeAnObstacle.erase(iWantToBeAnObstacle.begin() + j);
        }
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

  for(auto obstacle = iWantToBeAnObstacle.begin(); obstacle != iWantToBeAnObstacle.end();)
  {
    ASSERT(obstacle->seenCount >= 0 && obstacle->seenCount <= maxPercepts);
    //Decrease seencount once per second (if lastMeasurement is one second old)
    if(theCameraInfo.camera && obstacle->seenCount >= 1 && (obstacle->lastMeasurement + 1000) <= theFrameInfo.time)
    {
      obstacle->seenCount -= 1;
      obstacle->lastMeasurement = theFrameInfo.time;
    }
    if(obstacle->seenCount == 0 || //Delete old obstacles
       obstacle->center.squareAbs() >= MAX_DIST_ON_FIELD2 || obstacle->velocity.abs() > maxVelocity ||
       (obstacle->type == InternalExpObstacle::US && (obstacle->lastMeasurement + 1000) <= theFrameInfo.time) //US obstacles are not that trustworthy
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

void ExpOMProvider::addGoalPercepts()
{
  merged.clear();
  merged.resize(iWantToBeAnObstacle.size());
  for(const auto& goalPost : theGoalPercept.goalPosts)
  {
    if(goalPost.positionOnField.squareAbs() >= MAX_DIST_ON_FIELD2)
      continue;
    const Matrix2x2<> cov = getCovOfPointInWorld(goalPost.positionOnField);
    unsigned confidence = 1;
    if(goalPost.position != GoalPost::IS_UNKNOWN)
      confidence = deleteValue / 2;
    if(theGoalPercept.timeWhenCompleteGoalLastSeen == theFrameInfo.time)
      confidence = deleteValue;
    tryToMerge(InternalExpObstacle(cov, goalPost.positionOnField, InternalExpObstacle::GOALPOST,
               theFrameInfo.time, confidence, 0.f));
  }
}

Matrix2x2<> ExpOMProvider::getCovOfPointInWorld(const Vector2<>& pointInWorld2) const
{
  Vector3<> unscaledVectorToPoint = theCameraMatrix.invert() * Vector3<>(pointInWorld2.x, pointInWorld2.y, 0.f);
  const Vector3<> unscaledWorld = theCameraMatrix.rotation * unscaledVectorToPoint;
  const float scale = theCameraMatrix.translation.z / -unscaledWorld.z;
  Vector2<> pointInWorld(unscaledWorld.x * scale, unscaledWorld.y * scale);
  const float distance = pointInWorld.abs();
  Vector2<> cossin = distance == 0.f ? Vector2<>(1.f, 0.f) : pointInWorld * (1.f / distance);
  Matrix2x2<> rot(cossin, Vector2<>(-cossin.y, cossin.x));
  const Vector2<>& robotRotationDeviation = (theMotionInfo.motion == MotionRequest::stand)
      ? pRobotRotationDeviationInStand : pRobotRotationDeviation;
  Matrix2x2<> cov(Vector2<>(sqr(theCameraMatrix.translation.z / std::tan((distance == 0.f ? pi_2 : std::atan(theCameraMatrix.translation.z / distance))
                                - robotRotationDeviation.x) - distance), 0.f), Vector2<>(0.f, sqr(std::tan(robotRotationDeviation.y) * distance)));
  return rot * cov * rot.transpose();
}

void ExpOMProvider::propagateObstacles(ExpObstacleModel& expObstacleModel)
{
  expObstacleModel.eobs.clear();
  for(const auto& ob : iWantToBeAnObstacle)
  {
    if(debug || ob.seenCount >= minPercepts)
      expObstacleModel.eobs.emplace_back(ob);
  }
}

void ExpOMProvider::tryToMerge(const InternalExpObstacle& obstacle)
{
  if(iWantToBeAnObstacle.empty())
  {
    iWantToBeAnObstacle.emplace_back(obstacle);
    merged.push_back(true);
    return;
  }
  float distance = 0.f;
  const float thisMergeDistance = obstacle.type == InternalExpObstacle::GOALPOST ? goalMergeDistanceSqr : mergeDistanceSqr;
  float possibleMergeDist = MAX_DIST_ON_FIELD2;
  unsigned atMerge = 0; //element matching the merge condition
  for(int i = 0; i < static_cast<int>(iWantToBeAnObstacle.size()); ++i)
  {
    distance = (obstacle.center - iWantToBeAnObstacle[i].center).squareAbs();
    if(distance <= thisMergeDistance && !merged[i]) //found probably matching obstacle
    {
      if(iWantToBeAnObstacle[i].type == InternalExpObstacle::GOALPOST && obstacle.type != InternalExpObstacle::GOALPOST)
      {
        if(distance >= minimalDistancetoGoalpostSqr) //measured (maybe) a robot
          continue;

        if(iWantToBeAnObstacle[i].lastMeasurement == theFrameInfo.time)
          return;

        continue;
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
    // Avoid merging ultrasound with other measurements. Just drop them because the other measurements are much more precise.
    // Possible improvement: Use distance, but ignore direction.
    if(obstacle.type != InternalExpObstacle::US || iWantToBeAnObstacle[atMerge].type == InternalExpObstacle::US)
    {
      considerType(obstacle, iWantToBeAnObstacle[atMerge]);

      measurement(iWantToBeAnObstacle[atMerge], obstacle); //EKF
    }
    iWantToBeAnObstacle[atMerge].lastMeasurement = obstacle.lastMeasurement;
    iWantToBeAnObstacle[atMerge].seenCount = std::min(iWantToBeAnObstacle[atMerge].seenCount += obstacle.seenCount, maxPercepts);

    merged[atMerge] = true;
  }
  else
  {
    iWantToBeAnObstacle.emplace_back(obstacle);
    merged.push_back(true);
  }
}

void ExpOMProvider::dynamic(InternalExpObstacle& obstacle)
{
  float thisPNv2 = pNv2;
  //update the state
  const float rot = -theOdometer.odometryOffset.rotation; //obstacle has to move in the opposite direction
  const Vector2<> translation = theOdometer.odometryOffset.translation * -1.f;
  const Matrix2x2<> rotationMatrix(std::cos(rot), -std::sin(rot), std::sin(rot), std::cos(rot)); //common 2D rotation
  obstacle.center = rotationMatrix * obstacle.center;
  if(obstacle.velocity.abs() <= minVelocity || obstacle.type == InternalExpObstacle::GOALPOST)
  {
    obstacle.velocity.x = 0.f;
    obstacle.velocity.y = 0.f;
    obstacle.center += translation;

    for(int i = 2; i < 4; ++i)
      for(int j = 2; j < 4; ++j)
        obstacle.covariance[i][j] = 0.f;

    thisPNv2 = 0.f;
  }
  else
  {
    obstacle.velocity = rotationMatrix * obstacle.velocity;
    obstacle.center += translation + obstacle.velocity * deltaTime;
  }
  //compute covariance matrix
  //create jacobian matrix (column-major order)
  const Matrix4x4<> jacobian(
    Vector4<>(std::cos(rot), std::sin(rot), 0.f, 0.f),
    Vector4<>(-(std::sin(rot)), std::cos(rot), 0.f, 0.f),
    Vector4<>(deltaTime, 0.f, std::cos(rot), std::sin(rot)),
    Vector4<>(0.f, deltaTime, -(std::sin(rot)), std::cos(rot))
  );
  //process new covariance matrix
  //noise
  Matrix4x4<> processNoise(
    Vector4<>(pNp2, 0.f, 0.f, 0.f),
    Vector4<>(0.f, pNp2, 0.f, 0.f),
    Vector4<>(0.f, 0.f, thisPNv2, 0.f),
    Vector4<>(0.f, 0.f, 0.f, thisPNv2));
  obstacle.covariance = jacobian * obstacle.covariance * jacobian.transpose() + processNoise;
}

void ExpOMProvider::measurement(InternalExpObstacle& obstacle, const InternalExpObstacle& z)
{
  //innovation
  const Eigen::Vector2f innovation(z.center.x - obstacle.center.x, z.center.y - obstacle.center.y);
  //jacobian matrix
  Eigen::Matrix<float, 2, 4> jacobian;
  jacobian << 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f;

  //coariance matrices
  const Matrix2x2<> covOfZ = getCovOfPointInWorld(z.center);
  Eigen::Matrix4f covX;
  Eigen::Matrix2f covZ = Eigen::Matrix2f::Zero();
  //conversion
  for(int i = 0; i < 4; ++i)
  {
    for(int j = 0; j < 4; ++j)
    {
      covX(j, i) = obstacle.covariance[i][j];
      if(j < 2 && i < 2)
        covZ(j, i) = covOfZ[i][j];
    }
  }
  //computation of kalman gain, new state and covariance matrix
  Eigen::Matrix<float, 4, 2> CXZ;
  CXZ.noalias() = covX * jacobian.transpose();
  Eigen::Matrix2f CZZ;
  CZZ.noalias() = jacobian * CXZ + covZ;
  Eigen::Matrix<float, 4, 2> K;
  K.noalias() = CXZ * CZZ.inverse();
  Eigen::Vector4f muX(obstacle.center.x, obstacle.center.y, obstacle.velocity.x, obstacle.velocity.y);
  muX.noalias() += K * innovation;
  covX -= K * jacobian * covX;
  //conversion
  for(int i = 0; i < 4; ++i)
  {
    for(int j = 0; j < 4; ++j)
      obstacle.covariance[i][j] = covX(j, i);
  }

  obstacle.center.x = muX(0);
  obstacle.center.y = muX(1);
  obstacle.velocity.x = muX(2);
  obstacle.velocity.y = muX(3);
}

void ExpOMProvider::considerType(const InternalExpObstacle& z, InternalExpObstacle& old)
{
  if(old.type == z.type || z.type == InternalExpObstacle::UNKNOWN || z.type == InternalExpObstacle::US)
    return;
  if(old.type == InternalExpObstacle::UNKNOWN || old.type == InternalExpObstacle::US)
  {
    old.type = z.type;
    return;
  }
  if(old.type == InternalExpObstacle::GOALPOST || z.type == InternalExpObstacle::US)
    return;
  if(z.type == InternalExpObstacle::GOALPOST)
  {
    old.type = InternalExpObstacle::GOALPOST;
    return;
  }

  //here begins weird stuff
  //the following code should perfectly consider whether a robot is fallen/upright and red/blue
  old.color = std::min(colorThreshold * colorThreshold, std::max(z.color + old.color, -colorThreshold * colorThreshold));
  old.upright = std::min(2 * uprightThreshold, std::max(z.upright + old.upright, -2 * uprightThreshold));

  if(old.color < colorThreshold && old.color > -colorThreshold)
  {
    if(old.upright <= -uprightThreshold)
    {
      old.type = InternalExpObstacle::SOMEFALLENROBOT;
      return;
    }
    old.type = InternalExpObstacle::SOMEROBOT;
    return;
  }

  if(old.color >= colorThreshold)
  {
    if(old.upright <= -uprightThreshold)
    {
      old.type = InternalExpObstacle::FALLENRED;
      return;
    }
    old.type = InternalExpObstacle::ROBOTRED;
    return;
  }
  else
  {
    if(old.upright <= -uprightThreshold)
    {
      old.type = InternalExpObstacle::FALLENBLUE;
      return;
    }
    old.type = InternalExpObstacle::ROBOTBLUE;
    return;
  }

  old.type = z.type;
}

InternalExpObstacle::InternalExpObstacle(const Matrix2x2<> cov, const Vector2<> pCenter, const Type t, const unsigned time, const unsigned sC, const float varianceVel)
{
  covariance = Matrix4x4<>(Vector4<>(cov.c[0].x, cov.c[0].y, 0.f, 0.f), Vector4<>(cov.c[1].x, cov.c[1].y, 0.f, 0.f),
                           Vector4<>(0.f, 0.f, varianceVel, 0.f), Vector4<>(0.f, 0.f, 0.f, varianceVel));
  center = pCenter;
  type = t;
  lastMeasurement = time;
  velocity = Vector2<>(0.f, 0.f);
  seenCount = sC;
  upright = type < SOMEFALLENROBOT ? 1 : -1;
  if(type == FALLENBLUE || type == ROBOTBLUE)
    color = -1;
  else if(type == FALLENRED || type == ROBOTRED)
    color = 1;
  else
    color = 0;
}

InternalExpObstacle::InternalExpObstacle(const Matrix2x2<> cov, const Vector2<> pCenter, const unsigned time, const unsigned sC, const float varianceVel)
{
  covariance = Matrix4x4<>(Vector4<>(cov.c[0].x, cov.c[0].y, 0.f, 0.f), Vector4<>(cov.c[1].x, cov.c[1].y, 0.f, 0.f),
                           Vector4<>(0.f, 0.f, varianceVel, 0.f), Vector4<>(0.f, 0.f, 0.f, varianceVel));
  center = pCenter;
  type = UNKNOWN;
  lastMeasurement = time;
  velocity = Vector2<>(0.f, 0.f);
  seenCount = sC;
  upright = type < SOMEFALLENROBOT ? 1 : -1;
  if(type == FALLENBLUE || type == ROBOTBLUE)
    color = -1;
  else if(type == FALLENRED || type == ROBOTRED)
    color = 1;
  else
    color = 0;
}

void ExpOMProvider::mergeOverlapping()
{
  const float overlap = (2 * robotDepth) * (2 * robotDepth);
  for(int i = 0; i < static_cast<int>(iWantToBeAnObstacle.size()); ++i)
  {
    for(int j = static_cast<int>(iWantToBeAnObstacle.size()) - 1; j > i; --j)
    {
      if((iWantToBeAnObstacle[j].center - iWantToBeAnObstacle[i].center).squareAbs() <= overlap &&
         ((iWantToBeAnObstacle[i].type >= InternalExpObstacle::UNKNOWN && iWantToBeAnObstacle[j].type >= InternalExpObstacle::UNKNOWN)
         || iWantToBeAnObstacle[i].type == iWantToBeAnObstacle[j].type))
      {
        measurement(iWantToBeAnObstacle[i], iWantToBeAnObstacle[j]); //EKF

        iWantToBeAnObstacle[i].lastMeasurement = std::max(iWantToBeAnObstacle[i].lastMeasurement, iWantToBeAnObstacle[j].lastMeasurement);
        iWantToBeAnObstacle[i].seenCount = std::min(iWantToBeAnObstacle[i].seenCount += iWantToBeAnObstacle[j].seenCount, maxPercepts);
        iWantToBeAnObstacle.erase(iWantToBeAnObstacle.begin() + j);
      }
    }
  }
}
/*Mahalanobis distance cannot be used, due to bug in getCovOfPointInWorld
float ExpOMProvider::mahalanobisDistance(const InternalExpObstacle& one, const InternalExpObstacle& two) const
{
const Eigen::Vector2f diff(one.center.x - two.center.x, one.center.y - two.center.y);
if(diff.isZero(1.f))
return 0.f;
const Eigen::RowVector2f diffT(one.center.x - two.center.x, one.center.y - two.center.y);
Eigen::Matrix2f cov;
cov << one.covariance[0][0] + two.covariance[0][0], one.covariance[1][0] + two.covariance[1][0],
one.covariance[0][1] + two.covariance[0][1], one.covariance[1][1] + two.covariance[1][1];

Eigen::Matrix2f covInv;
bool invertible;
cov.computeInverseWithCheck(covInv, invertible);
ASSERT(invertible);
return std::sqrt(diffT * (covInv * diff));
}
*/