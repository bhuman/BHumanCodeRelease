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
}

void HeadMotionEngine::update(HeadMotionEngineOutput& headMotionEngineOutput)
{
  // update requested angles
  requestedPan = theHeadAngleRequest.pan;
  requestedTilt = theHeadAngleRequest.tilt;

  //
  float maxAcc = theGroundContactState.contact ? 10.f : 1.f; // arbitrary value that seems to be good...
  MODIFY("module:HeadMotionEngine:maxAcceleration", maxAcc);

  const float pan = requestedPan == JointAngles::off ? JointAngles::off : Rangef(theHeadLimits.minPan(), theHeadLimits.maxPan()).limit(requestedPan);
  const float tilt = requestedTilt == JointAngles::off ? JointAngles::off : theHeadLimits.getTiltBound(pan).limit(requestedTilt);

  constexpr float deltaTime = Constants::motionCycleTime;
  const Vector2f position(headMotionEngineOutput.pan == JointAngles::off ? theJointAngles.angles[Joints::headYaw] : headMotionEngineOutput.pan,
                          headMotionEngineOutput.tilt == JointAngles::off ? theJointAngles.angles[Joints::headPitch] : headMotionEngineOutput.tilt);
  const Vector2f target(pan == JointAngles::off ? 0.f : pan, tilt == JointAngles::off ? 0.f : tilt);
  Vector2f offset(target - position);
  const float distanceToTarget = offset.norm();

  // calculate max speed
  const float maxSpeedForDistance = std::sqrt(2.f * distanceToTarget * maxAcc * 0.8f);

  const float requestedSpeed = theHeadAngleRequest.stopAndGoMode
                               ? theHeadAngleRequest.speed * (std::cos(pi2 / stopAndGoModeFrequenzy * theFrameInfo.time) / 2.f + .5f)
                               : static_cast<float>(theHeadAngleRequest.speed);

  const float maxSpeed = std::min(maxSpeedForDistance, requestedSpeed);

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

  // set new position
  headMotionEngineOutput.pan = pan == JointAngles::off ? JointAngles::off : newPosition.x();
  headMotionEngineOutput.tilt = tilt == JointAngles::off ? JointAngles::off : newPosition.y();
  headMotionEngineOutput.moving = pan != JointAngles::off && tilt != JointAngles::off && ((newPosition - position) / deltaTime).squaredNorm() > sqr(maxAcc * deltaTime * 0.5f);

  // check reachability
  headMotionEngineOutput.reachable = true;
  if(pan != requestedPan || tilt != requestedTilt)
    headMotionEngineOutput.reachable = false;

  // store some values for the next iteration
  lastSpeed = speed;
}
