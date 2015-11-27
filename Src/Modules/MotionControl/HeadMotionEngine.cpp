/**
 * @file HeadMotionEngine.cpp
 * This file implements a module that creates head joint angles from desired head motion.
 * @author <a href="allli@informatik.uni-bremen.de">Alexander Härtl</a>
 * @author Colin Graf
 */

#include <algorithm>
#include "HeadMotionEngine.h"
#include "Tools/Range.h"
#include "Tools/Debugging/DebugDrawings.h" // PLOT

MAKE_MODULE(HeadMotionEngine, motionControl)

HeadMotionEngine::HeadMotionEngine()
{
  requestedPan = requestedTilt = JointAngles::off;
  lastSpeed = Vector2f::Zero();

  deathPoints[0] = Geometry::getCircle(
                     Vector2i(45, -25), Vector2i(120, -20), Vector2i(90, -17));
  deathPoints[1] = Geometry::getCircle(
                     Vector2i(-45, -25), Vector2i(-120, -20), Vector2i(-90, -17));
  deathPoints[2] = Geometry::getCircle(
                     Vector2i(17, 46), Vector2i(85, 23), Vector2i(120, 27));
  deathPoints[3] = Geometry::getCircle(
                     Vector2i(-17, 46), Vector2i(-85, 23), Vector2i(-120, 27));

  for(int i = 0; i < 4; ++i)
  {
    deathPoints[i].center.x() = Angle::fromDegrees(deathPoints[i].center.x());
    deathPoints[i].center.y() = Angle::fromDegrees(deathPoints[i].center.y());
    deathPoints[i].radius = Angle::fromDegrees(deathPoints[i].radius) * 1.015f;
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

  float pan = requestedPan == JointAngles::off ? JointAngles::off : Rangef(theJointCalibration.joints[Joints::headYaw].minAngle, theJointCalibration.joints[Joints::headYaw].maxAngle).limit(requestedPan);
  float tilt = requestedTilt == JointAngles::off ? JointAngles::off : Rangef(theJointCalibration.joints[Joints::headPitch].minAngle, theJointCalibration.joints[Joints::headPitch].maxAngle).limit(requestedTilt);

  const float deltaTime = theFrameInfo.cycleTime;
  const Vector2f position(headJointRequest.pan == JointAngles::off ? theJointAngles.angles[Joints::headYaw] : headJointRequest.pan,
                           headJointRequest.tilt == JointAngles::off ? theJointAngles.angles[Joints::headPitch] : headJointRequest.tilt);
  const Vector2f target(pan == JointAngles::off ? 0 : pan, tilt == JointAngles::off ? 0 : tilt);
  Vector2f offset(target - position);
  const float distanceToTarget = offset.norm();

  // calculate max speed
  const float maxSpeedForDistance = std::sqrt(2.f * distanceToTarget * maxAcc * 0.8f);
  const float maxSpeed = std::min(maxSpeedForDistance, static_cast<float>(theHeadAngleRequest.speed));

  // max speed clipping
  if(distanceToTarget / deltaTime > maxSpeed)
    offset *= maxSpeed * deltaTime / distanceToTarget; //<=> offset.normalize(maxSpeed * deltaTime);

  // max acceleration clipping
  Vector2f speed(offset / deltaTime);
  Vector2f acc((speed - lastSpeed) / deltaTime);
  const float accSquareAbs = acc.squaredNorm();
  if(accSquareAbs > maxAcc * maxAcc)
  {
    acc *= maxAcc * deltaTime / std::sqrt(accSquareAbs);
    speed = acc + lastSpeed;
    offset = speed * deltaTime;
  }
  /* <=>
  Vector2f speed(offset / deltaTime);
  Vector2f acc((speed - lastSpeed) / deltaTime);
  if(acc.squaredNorm() > maxAcc * maxAcc)
  {
    speed = acc.normalize(maxAcc * deltaTime) + lastSpeed;
    offset = speed * deltaTime;
  }
  */
  PLOT("module:HeadMotionEngine:speed", toDegrees(speed.norm()));

  // calculate new position
  Vector2f newPosition(position + offset);

  // make sure we don't get to close to the evil points of death
  if(pan != JointAngles::off && tilt != JointAngles::off)
    for(int i = 0; i < 4; ++i)
    {
      Vector2f deathPointToPosition(newPosition - deathPoints[i].center);
      const float deathPointToPositionSquareAbs = deathPointToPosition.squaredNorm();
      if(deathPointToPositionSquareAbs != 0.f && deathPointToPositionSquareAbs < sqr(deathPoints[i].radius))
      {
        const float deathPointToPositionAbs = std::sqrt(deathPointToPositionSquareAbs);
        deathPointToPosition *= (deathPoints[i].radius - deathPointToPositionAbs) / deathPointToPositionAbs;
        newPosition += deathPointToPosition;
      }
    }

  // set new position
  headJointRequest.pan = pan == JointAngles::off ? JointAngles::off : newPosition.x();
  headJointRequest.tilt = tilt == JointAngles::off ? JointAngles::off : newPosition.y();
  headJointRequest.moving = pan != JointAngles::off && tilt != JointAngles::off && ((newPosition - position) / deltaTime).squaredNorm() > sqr(maxAcc * deltaTime * 0.5f);

  // check reachability
  headJointRequest.reachable = true;
  if(pan != requestedPan || tilt != requestedTilt)
    headJointRequest.reachable = false;
  else
    for(int i = 0; i < 4; ++i)
      if((target - deathPoints[i].center).squaredNorm() < sqr(deathPoints[i].radius))
        headJointRequest.reachable = false;

  // store some values for the next iteration
  lastSpeed = speed;
}
