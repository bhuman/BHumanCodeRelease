/**
 * @file HeadMotionEngine.cpp
 * This file implements a module that creates head joint angles from desired head motion.
 * @author <a href="allli@informatik.uni-bremen.de">Alexander Härtl</a>
 * @author Colin Graf
 */

#include "HeadMotionEngine.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Range.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(HeadMotionEngine, motionControl);

void HeadMotionEngine::update(HeadMotionGenerator& headMotionGenerator)
{
  DECLARE_PLOT("module:HeadMotionEngine:speed");

  headMotionGenerator.calcJoints = [this](bool setJoints, JointRequest& jointRequest, HeadMotionInfo& headMotionInfo)
  {
    if(!setJoints)
    {
      // Do not write anything to the joint request.
      headMotionInfo.moving = false;
      lastSpeed = Vector2f::Zero();
      return;
    }

    // Set angles directly when calibration flag is set in HeadAngleRequest
    if(theHeadAngleRequest.disableClippingAndInterpolation)
    {
      headMotionInfo.moving = true;
      lastSpeed = Vector2f::Zero();
      jointRequest.angles[Joints::headYaw] = theHeadAngleRequest.pan;
      jointRequest.angles[Joints::headPitch] = theHeadAngleRequest.tilt;
      jointRequest.stiffnessData.stiffnesses[Joints::headYaw] = 100;
      jointRequest.stiffnessData.stiffnesses[Joints::headPitch] = 100;
      return;
    }

    const Angle requestedPan = theHeadAngleRequest.pan;
    const Angle requestedTilt = theHeadAngleRequest.tilt;

    const float maxAcc = theGroundContactState.contact ? maxAcceleration : maxAccelerationNoGroundContact;

    const float pan = requestedPan == JointAngles::off ? static_cast<float>(JointAngles::off) : Rangef(theHeadLimits.minPan(), theHeadLimits.maxPan()).limit(requestedPan);
    const float tilt = requestedTilt == JointAngles::off ? JointAngles::off : theHeadLimits.getTiltBound(pan).limit(requestedTilt);

    constexpr float deltaTime = Constants::motionCycleTime;
    const Vector2f position(theJointRequest.stiffnessData.stiffnesses[Joints::headYaw] == 0 ? theJointAngles.angles[Joints::headYaw] : theJointRequest.angles[Joints::headYaw],
                            theJointRequest.stiffnessData.stiffnesses[Joints::headPitch] == 0 ? theJointAngles.angles[Joints::headPitch] : theJointRequest.angles[Joints::headPitch]);
    const Vector2f target(pan == JointAngles::off ? 0.f : pan, tilt == JointAngles::off ? 0.f : tilt);
    Vector2f offset(target - position);
    const float distanceToTarget = offset.norm();

    // calculate max speed
    const float maxSpeedForDistance = std::sqrt(2.f * distanceToTarget * maxAcc * 0.8f);

    const float requestedSpeed = theHeadAngleRequest.stopAndGoMode
                                 ? theHeadAngleRequest.speed * (std::cos(pi2 / stopAndGoModeFrequency * theFrameInfo.time) / 2.f + .5f)
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
    jointRequest.angles[Joints::headYaw] = pan == JointAngles::off ? JointAngles::off : static_cast<Angle>(newPosition.x());
    jointRequest.angles[Joints::headPitch] = tilt == JointAngles::off ? JointAngles::off : static_cast<Angle>(newPosition.y());
    headMotionInfo.moving = pan != JointAngles::off && tilt != JointAngles::off && ((newPosition - position) / deltaTime).squaredNorm() > sqr(maxAcc * deltaTime * 0.5f);

    // store some values for the next iteration
    lastSpeed = speed;
  };
}
