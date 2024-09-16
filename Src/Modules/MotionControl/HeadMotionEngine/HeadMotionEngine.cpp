/**
 * @file HeadMotionEngine.cpp
 * This file implements a module that creates head joint angles from desired head motion.
 * @author <a href="allli@informatik.uni-bremen.de">Alexander Härtl</a>
 * @author Colin Graf
 * @author Felix Wenk
 * @author Andreas Stolpmann
 */

#include "HeadMotionEngine.h"
#include "Debugging/Plot.h"
#include "Math/Geometry.h"
#include "Math/Range.h"
#include "Math/BHMath.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(HeadMotionEngine);

HeadMotionEngine::HeadMotionEngine()
{
  panBounds.min = theHeadLimits.minPan();
  panBounds.max = theHeadLimits.maxPan();
}

void HeadMotionEngine::update(HeadMotionGenerator& headMotionGenerator)
{
  DECLARE_PLOT("module:HeadMotionEngine:speed");

  updateHeadAngleRequest(theHeadAngleRequest, lastWasLower);

  headMotionGenerator.calcJoints = [this](bool setJoints, JointRequest& jointRequest, HeadMotionInfo& headMotionInfo,
                                          const MotionRequest& motionRequest, const OdometryData& odometry)
  {
    ASSERT(theHeadAngleRequest.pan != JointAngles::ignore);
    ASSERT(theHeadAngleRequest.tilt != JointAngles::ignore);
    if(!setJoints)
    {
      // Do not write anything to the joint request.
      headMotionInfo.moving = false;
      lastSpeed = Vector2f::Zero();
      return;
    }

    if(theHeadAngleRequest.pan != JointAngles::off)
      theHeadAngleRequest.pan += (odometry.inverse() * motionRequest.odometryData).rotation;

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

void HeadMotionEngine::updateHeadAngleRequest(HeadAngleRequest& headAngleRequest, bool& lastWasLower) const
{
  Vector2a panTiltUpperCam;
  Vector2a panTiltLowerCam;

  if(theHeadMotionRequest.mode == HeadMotionRequest::calibrationMode)
  {
    // Pass the given angles without further checking.
    headAngleRequest.pan = theHeadMotionRequest.pan;
    headAngleRequest.tilt = theHeadMotionRequest.tilt;
    headAngleRequest.speed = theHeadMotionRequest.speed;
    // Do not clip angles.
    headAngleRequest.disableClippingAndInterpolation = true;
    // Handle ignore
    if(theHeadMotionRequest.tilt == JointAngles::ignore)
      headAngleRequest.tilt = theJointRequest.angles[Joints::headPitch];
    if(theHeadMotionRequest.pan == JointAngles::ignore)
      headAngleRequest.pan = theJointRequest.angles[Joints::headYaw] - theOdometryDataPreview.odometryChange.rotation;
    return;
  }
  else
    headAngleRequest.disableClippingAndInterpolation = false;

  if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode)
  {
    panTiltUpperCam.x() = theHeadMotionRequest.pan;
    panTiltUpperCam.y() = theHeadMotionRequest.tilt - theRobotDimensions.getTiltNeckToCamera(false);
    panTiltLowerCam.x() = theHeadMotionRequest.pan;
    panTiltLowerCam.y() = theHeadMotionRequest.tilt - theRobotDimensions.getTiltNeckToCamera(true);

    if(theHeadMotionRequest.tilt == JointAngles::off || theHeadMotionRequest.tilt == JointAngles::ignore)
      panTiltUpperCam.y() = theJointRequest.angles[Joints::headPitch];
    if(theHeadMotionRequest.tilt == JointAngles::off || theHeadMotionRequest.tilt == JointAngles::ignore)
      panTiltLowerCam.y() = theJointRequest.angles[Joints::headPitch];

    if(theHeadMotionRequest.pan == JointAngles::off || theHeadMotionRequest.pan == JointAngles::ignore)
      panTiltUpperCam.x() = theJointRequest.angles[Joints::headYaw] - theOdometryDataPreview.odometryChange.rotation;
    if(theHeadMotionRequest.pan == JointAngles::off || theHeadMotionRequest.pan == JointAngles::ignore)
      panTiltLowerCam.x() = theJointRequest.angles[Joints::headYaw] - theOdometryDataPreview.odometryChange.rotation;
  }
  else
  {
    Pose3f base(theHeadMotionRequest.mode == HeadMotionRequest::targetMode
                ? Pose3f() : static_cast<const Pose3f&>(theTorsoMatrix));
    const Vector3f hip2Target = base
                                .rotateY(theCameraCalibration.bodyRotationCorrection.y())
                                .rotateX(theCameraCalibration.bodyRotationCorrection.x())
                                .inverse() * theHeadMotionRequest.target;

    calculatePanTiltAngles(hip2Target, CameraInfo::upper, panTiltUpperCam);
    calculatePanTiltAngles(hip2Target, CameraInfo::lower, panTiltLowerCam);
  }

  if(panTiltUpperCam.x() < panBounds.min)
  {
    panTiltUpperCam.x() = panBounds.min;
    panTiltLowerCam.x() = panBounds.min;
  }
  else if(panTiltUpperCam.x() > panBounds.max)
  {
    panTiltUpperCam.x() = panBounds.max;
    panTiltLowerCam.x() = panBounds.max;
  }

  Rangea tiltBoundUpperCam = theHeadLimits.getTiltBound(panTiltUpperCam.x());
  Rangea tiltBoundLowerCam = theHeadLimits.getTiltBound(panTiltLowerCam.x());

  adjustTiltBoundToShoulder(panTiltUpperCam.x(), CameraInfo::upper, tiltBoundUpperCam);
  adjustTiltBoundToShoulder(panTiltLowerCam.x(), CameraInfo::lower, tiltBoundLowerCam);

  bool lowerCam = false;
  headAngleRequest.pan = panTiltUpperCam.x(); // Pan is the same for both cams

  if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::upperCamera)
  {
    headAngleRequest.tilt = panTiltUpperCam.y();
    lowerCam = false;
  }
  else if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::lowerCamera)
  {
    headAngleRequest.tilt = panTiltLowerCam.y();
    lowerCam = true;
  }
  else
  {
    if(theHeadMotionRequest.mode != HeadMotionRequest::panTiltMode)
    {
      const Angle useLowerCamThreshold = !lastWasLower ? lowerCamThreshold.min : lowerCamThreshold.max;
      const Angle lowerCamOffset = defaultTilt - theCameraIntrinsics.cameras[CameraInfo::lower].openingAngleHeight / 3.f + theCameraCalibration.bodyRotationCorrection.y() + theCameraCalibration.cameraRotationCorrections[CameraInfo::lower].y() + useLowerCamThreshold;
      if(panTiltUpperCam.y() > tiltBoundUpperCam.max || panTiltLowerCam.y() > lowerCamOffset)
      {
        headAngleRequest.tilt = panTiltLowerCam.y();
        lowerCam = true;
      }
      else
        headAngleRequest.tilt = panTiltUpperCam.y();
    }
    else
    {
      headAngleRequest.tilt = panTiltUpperCam.y();
      lowerCam = false;
    }
  }

  lastWasLower = lowerCam;

  if(lowerCam)
    tiltBoundLowerCam.clamp(headAngleRequest.tilt);
  else
    tiltBoundUpperCam.clamp(headAngleRequest.tilt);

  if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode)
  {
    if(theHeadMotionRequest.tilt == JointAngles::off)
      headAngleRequest.tilt = JointAngles::off;
    if(theHeadMotionRequest.pan == JointAngles::off)
      headAngleRequest.pan = JointAngles::off;
  }
  headAngleRequest.speed = theHeadMotionRequest.speed;
  headAngleRequest.stopAndGoMode = theHeadMotionRequest.stopAndGoMode;
}

void HeadMotionEngine::calculatePanTiltAngles(const Vector3f& hip2Target, CameraInfo::Camera camera, Vector2a& panTilt) const
{
  InverseKinematic::calcHeadJoints(hip2Target, pi_2, theRobotDimensions, camera, panTilt, theCameraCalibration);
}

void HeadMotionEngine::adjustTiltBoundToShoulder(const Angle pan, CameraInfo::Camera camera, Range<Angle>& tiltBound) const
{
  const Limbs::Limb shoulder = pan > 0_deg ? Limbs::shoulderLeft : Limbs::shoulderRight;
  const Vector3f& shoulderVector = theRobotModel.limbs[shoulder].translation;
  const RobotCameraMatrix rcm(theRobotDimensions, pan, 0.0f, theCameraCalibration, camera);
  Vector3f intersection = Vector3f::Zero();
  if(theHeadLimits.intersectionWithShoulderEdge(rcm, shoulderVector, intersection))
  {
    Vector2a intersectionPanTilt;
    calculatePanTiltAngles(intersection, camera, intersectionPanTilt);
    if(intersectionPanTilt.y() < tiltBound.max) // if(tilt smaller than upper bound)
      tiltBound.max = intersectionPanTilt.y();
  }
}
