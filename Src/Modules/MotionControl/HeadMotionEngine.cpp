/**
* @file HeadMotionEngine.cpp
* This file implements a module that creates head joint angles from desired head motion.
* @author <a href="allli@informatik.uni-bremen.de">Alexander Härtl</a>
* @author Colin Graf
*/

#include <algorithm>
#include "HeadMotionEngine.h"
#include "Tools/Math/Common.h"
#include "Tools/Range.h"
#include "Tools/Debugging/DebugDrawings.h" // PLOT

MAKE_MODULE(HeadMotionEngine, Motion Control)

HeadMotionEngine::HeadMotionEngine()
{
  requestedPan = requestedTilt = JointData::off;
  lastSpeed = Vector2<>();

  deathPoints[0] = Geometry::getCircle(
                     Vector2<int>(45, -25), Vector2<int>(120, -20), Vector2<int>(90, -17));
  deathPoints[1] = Geometry::getCircle(
                     Vector2<int>(-45, -25), Vector2<int>(-120, -20), Vector2<int>(-90, -17));
  deathPoints[2] = Geometry::getCircle(
                     Vector2<int>(17, 46), Vector2<int>(85, 23), Vector2<int>(120, 27));
  deathPoints[3] = Geometry::getCircle(
                     Vector2<int>(-17, 46), Vector2<int>(-85, 23), Vector2<int>(-120, 27));

  for(int i = 0; i < 4; ++i)
  {
    deathPoints[i].center.x = fromDegrees(deathPoints[i].center.x);
    deathPoints[i].center.y = fromDegrees(deathPoints[i].center.y);
    deathPoints[i].radius = fromDegrees(deathPoints[i].radius) * 1.015f;
  }
}

void HeadMotionEngine::update(HeadJointRequest& headJointRequest)
{
  // update requested angles
  requestedPan = theHeadAngleRequest.pan;
  requestedTilt = theHeadAngleRequest.tilt;

  //
  float maxAcc = theGroundContactState.contact ? 10.f : 1.f; // arbitrary value that seems to be good...
  MODIFY("module:HeadMotionEngine:maxAcceleration", maxAcc);

  float pan = requestedPan == JointData::off ? JointData::off : Range<>(theJointCalibration.joints[JointData::HeadYaw].minAngle, theJointCalibration.joints[JointData::HeadYaw].maxAngle).limit(requestedPan);
  float tilt = requestedTilt == JointData::off ? JointData::off : Range<>(theJointCalibration.joints[JointData::HeadPitch].minAngle, theJointCalibration.joints[JointData::HeadPitch].maxAngle).limit(requestedTilt);

  const float deltaTime = theFrameInfo.cycleTime;
  const Vector2<> position(headJointRequest.pan == JointData::off ? theFilteredJointData.angles[JointData::HeadYaw] : headJointRequest.pan,
                           headJointRequest.tilt == JointData::off ? theFilteredJointData.angles[JointData::HeadPitch] : headJointRequest.tilt);
  const Vector2<> target(pan == JointData::off ? 0 : pan, tilt == JointData::off ? 0 : tilt);
  Vector2<> offset(target - position);
  const float distanceToTarget = offset.abs();

  // calculate max speed
  const float maxSpeedForDistance = std::sqrt(2.f * distanceToTarget * maxAcc * 0.8f);
  const float maxSpeed = std::min(maxSpeedForDistance, theHeadAngleRequest.speed);

  // max speed clipping
  if(distanceToTarget / deltaTime > maxSpeed)
    offset *= maxSpeed * deltaTime / distanceToTarget; //<=> offset.normalize(maxSpeed * deltaTime);

  // max acceleration clipping
  Vector2<> speed(offset / deltaTime);
  Vector2<> acc((speed - lastSpeed) / deltaTime);
  const float accSquareAbs = acc.squareAbs();
  if(accSquareAbs > maxAcc * maxAcc)
  {
    acc *= maxAcc * deltaTime / std::sqrt(accSquareAbs);
    speed = acc + lastSpeed;
    offset = speed * deltaTime;
  }
  /* <=>
  Vector2<> speed(offset / deltaTime);
  Vector2<> acc((speed - lastSpeed) / deltaTime);
  if(acc.squareAbs() > maxAcc * maxAcc)
  {
    speed = acc.normalize(maxAcc * deltaTime) + lastSpeed;
    offset = speed * deltaTime;
  }
  */
  PLOT("module:HeadMotionEngine:speed", toDegrees(speed.abs()));

  // calculate new position
  Vector2<> newPosition(position + offset);

  // make sure we don't get to close to the evil points of death
  if(pan != JointData::off && tilt != JointData::off)
    for(int i = 0; i < 4; ++i)
    {
      Vector2<> deathPointToPosition(newPosition - deathPoints[i].center);
      const float deathPointToPositionSquareAbs = deathPointToPosition.squareAbs();
      if(deathPointToPositionSquareAbs != 0.f && deathPointToPositionSquareAbs < sqr(deathPoints[i].radius))
      {
        const float deathPointToPositionAbs = std::sqrt(deathPointToPositionSquareAbs);
        deathPointToPosition *= (deathPoints[i].radius - deathPointToPositionAbs) / deathPointToPositionAbs;
        newPosition += deathPointToPosition;
      }
    }

  // set new position
  headJointRequest.pan = pan == JointData::off ? JointData::off : newPosition.x;
  headJointRequest.tilt = tilt == JointData::off ? JointData::off : newPosition.y;
  headJointRequest.moving = pan != JointData::off && tilt != JointData::off && ((newPosition - position) / deltaTime).squareAbs() > sqr(maxAcc * deltaTime * 0.5f);

  // check reachability
  headJointRequest.reachable = true;
  if(pan != requestedPan || tilt != requestedTilt)
    headJointRequest.reachable = false;
  else
    for(int i = 0; i < 4; ++i)
      if((target - deathPoints[i].center).squareAbs() < sqr(deathPoints[i].radius))
        headJointRequest.reachable = false;

  // store some values for the next iteration
  lastSpeed = speed;
}
