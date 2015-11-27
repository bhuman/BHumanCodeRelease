/**
* @file CameraControlEngine.cpp
* @author Andreas Stolpmann
* (Based on an old version by Felix Wenk)
*/

#include "CameraControlEngine.h"
#include "Tools/Motion/InverseKinematic.h"

MAKE_MODULE(CameraControlEngine, behaviorControl);

CameraControlEngine::CameraControlEngine()
{
  minPan = theHeadLimits.minPan();
  maxPan = theHeadLimits.maxPan();
}

void CameraControlEngine::update(HeadAngleRequest& headAngleRequest)
{
  Vector2f panTiltUpperCam;
  Vector2f panTiltLowerCam;

  if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode)
  {
    panTiltUpperCam.x() = theHeadMotionRequest.pan;
    panTiltUpperCam.y() = theHeadMotionRequest.tilt + theRobotDimensions.getTiltNeckToCamera(false);
    panTiltLowerCam.x() = theHeadMotionRequest.pan;
    panTiltLowerCam.y() = theHeadMotionRequest.tilt + theRobotDimensions.getTiltNeckToCamera(true);
  }
  else
  {
    Vector3f hip2Target;
    if(theHeadMotionRequest.mode == HeadMotionRequest::targetMode)
      hip2Target = theHeadMotionRequest.target;
    else
      hip2Target = theTorsoMatrix.inverse() * theHeadMotionRequest.target;

    calculatePanTiltAngles(hip2Target, false, panTiltUpperCam);
    calculatePanTiltAngles(hip2Target, true, panTiltLowerCam);

    if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::autoCamera)
      panTiltLowerCam.y() = defaultTilt;
  }

  if(panTiltUpperCam.x() < minPan)
  {
    panTiltUpperCam.x() = minPan;
    panTiltLowerCam.x() = minPan;
  }
  else if(panTiltUpperCam.x() > maxPan)
  {
    panTiltUpperCam.x() = maxPan;
    panTiltLowerCam.x() = maxPan;
  }

  Vector2f tiltBoundUpperCam = theHeadLimits.getTiltBound(panTiltUpperCam.x());
  Vector2f tiltBoundLowerCam = theHeadLimits.getTiltBound(panTiltLowerCam.x());

  adjustTiltBoundToShoulder(panTiltUpperCam.x(), false, tiltBoundUpperCam);
  adjustTiltBoundToShoulder(panTiltLowerCam.x(), true, tiltBoundLowerCam);

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
      if(panTiltLowerCam.y() > tiltBoundLowerCam.x())
      {
        headAngleRequest.tilt = panTiltUpperCam.y();
        lowerCam = false;
      }
      else if(panTiltUpperCam.y() < moveHeadThreshold)
      {
        headAngleRequest.tilt = defaultTilt - std::abs(moveHeadThreshold - panTiltUpperCam.y());
        lowerCam = false;
      }
      else
      {
        headAngleRequest.tilt = panTiltLowerCam.y();
        lowerCam = true;
      }
    }
    else
    {
      headAngleRequest.tilt = panTiltUpperCam.y();
      lowerCam = true;
    }
  }

  if(lowerCam)
  {
    if(headAngleRequest.tilt > tiltBoundLowerCam.x())
      headAngleRequest.tilt = tiltBoundLowerCam.x();
    if(headAngleRequest.tilt < tiltBoundLowerCam.y())
      headAngleRequest.tilt = tiltBoundLowerCam.y();
  }
  else
  {
    if(headAngleRequest.tilt > tiltBoundUpperCam.x())
      headAngleRequest.tilt = tiltBoundUpperCam.x();
    if(headAngleRequest.tilt < tiltBoundUpperCam.y())
      headAngleRequest.tilt = tiltBoundUpperCam.y();
  }

  if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode)
  {
    if(theHeadMotionRequest.tilt == JointAngles::off)
      headAngleRequest.tilt = JointAngles::off;
    if(theHeadMotionRequest.pan == JointAngles::off)
      headAngleRequest.pan = JointAngles::off;
  }
  headAngleRequest.speed = theHeadMotionRequest.speed;
}

void CameraControlEngine::calculatePanTiltAngles(const Vector3f& hip2Target, bool lowerCamera, Vector2f& panTilt) const
{
  InverseKinematic::calcHeadJoints(hip2Target, pi_2, theRobotDimensions, lowerCamera, panTilt, theCameraCalibration);
}

void CameraControlEngine::adjustTiltBoundToShoulder(const float pan, const bool lowerCamera, Vector2f& tiltBound) const
{
  Limbs::Limb shoulder = pan > 0.0f ? Limbs::shoulderLeft : Limbs::shoulderRight;
  const Vector3f& shoulderVector = theRobotModel.limbs[shoulder].translation;
  RobotCameraMatrix rcm(theRobotDimensions, pan, 0.0f, theCameraCalibration, !lowerCamera);
  Vector3f intersection = Vector3f::Zero();
  if(theHeadLimits.intersectionWithShoulderEdge(rcm, shoulderVector, intersection))
  {
    Vector2f intersectionPanTilt;
    calculatePanTiltAngles(intersection, lowerCamera, intersectionPanTilt);
    if(intersectionPanTilt.y() < tiltBound.x()) // if(tilt smaller than upper bound)
      tiltBound.x() = intersectionPanTilt.y();
  }
}
