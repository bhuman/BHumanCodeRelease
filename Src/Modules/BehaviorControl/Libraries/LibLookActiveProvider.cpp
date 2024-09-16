/**
 * @file LibLookActiveProvider.cpp
 * @author Andreas Stolpmann
 * @author Florian Scholz
 */

#include "LibLookActiveProvider.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/BehaviorControl/Strategy/ActiveRole.h"
#include "Debugging/Plot.h"
#include "Math/Geometry.h"

MAKE_MODULE(LibLookActiveProvider);

void LibLookActiveProvider::update(LibLookActive& libLookActive)
{
  // debug stuff
  translationSpeedBuffer.push_front(theMotionInfo.isMotion(MotionPhase::walk) ? theMotionInfo.speed.translation.norm() : 0.f);
  rotationSpeedBuffer.push_front(theMotionInfo.isMotion(MotionPhase::walk) ? Angle(std::abs(theMotionInfo.speed.rotation)) : 0_deg);

  DEBUG_RESPONSE("behavior:LibLookActiveProvider:stuff")
  {
    if(translationSpeedBuffer.full() && rotationSpeedBuffer.full())
    {
      calculateSpeedFactors();
      OUTPUT_TEXT("Translation: " << translationSpeedBuffer.average() << " Factor: " << translationSpeedFactor);
      OUTPUT_TEXT("Rotation: " << rotationSpeedBuffer.average() << " Factor: " << rotationSpeedFactor);
      OUTPUT_TEXT("Both: " << (translationSpeedFactor * rotationSpeedFactor));
    }
  }
  //update representation
  libLookActive.calculateHeadTarget = [this](const bool withBall, const bool ignoreBall, const bool onlyOwnBall, const bool fixTilt, const float slowdownFactor) -> HeadTarget
  {
    return calculateHeadTarget(withBall, ignoreBall, onlyOwnBall, fixTilt, slowdownFactor);
  };
}

HeadTarget LibLookActiveProvider::calculateHeadTarget(const bool withBall, const bool ignoreBall, const bool onlyOwnBall, const bool fixTilt, const float slowdownFactor)
{
  theBallPositionRelative = theBallModel.estimate.position;
  theBallSpeedRelative = theBallModel.estimate.velocity;

  teamBallIsUsed = false;

  if(!onlyOwnBall
     && (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > ballPositionUnknownTimeout || theFrameInfo.getTimeSince(theBallModel.timeWhenDisappeared) > 1000)
     && theTeammatesBallModel.isValid)
  {
    theBallPositionRelative = theRobotPose.inverse() * theTeammatesBallModel.position;
    theBallSpeedRelative = theTeammatesBallModel.velocity.rotated(-theRobotPose.rotation);

    teamBallIsUsed = true;
  }

  calculateSpeedFactors();
  const bool forceBall = !ignoreBall && !ballPositionUnknown(onlyOwnBall) && (withBall || shouldLookAtBall()) && ballAnglesReachable();

  const Angle pan = calculatePan(forceBall);
  PLOT("module:LibLookActiveProvider:pan", pan.toDegrees());
  const Angle tilt = fixTilt ? Angle(0.38f) : calculateTilt(forceBall, onlyOwnBall);
  PLOT("module:LibLookActiveProvider:tilt", tilt.toDegrees());
  const Angle speed = calculateSpeed(forceBall, pan, slowdownFactor);
  PLOT("module:LibLookActiveProvider:speed", speed.toDegrees());

  HeadTarget theTarget;
  theTarget.pan = pan;
  theTarget.tilt = tilt;
  theTarget.speed = speed;
  theTarget.cameraControlMode = HeadMotionRequest::CameraControlMode::upperCamera;
  theTarget.stopAndGoMode = !ignoreBall && theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > ballPositionUnknownTimeout;
  return theTarget;
}

Angle LibLookActiveProvider::calculatePan(const bool forceBall)
{
  // head moves in the following pattern:
  std::vector<Angle> basePanAngles;
  basePanAngles.emplace_back(largeDefaultPan);
  basePanAngles.emplace_back(-smallDefaultPan);
  basePanAngles.emplace_back(smallDefaultPan);
  basePanAngles.emplace_back(-largeDefaultPan);
  basePanAngles.emplace_back(smallDefaultPan);
  basePanAngles.emplace_back(-smallDefaultPan);

  for(Angle& pan : basePanAngles)
    pan = sgn(pan) * (minPanMoving + ((std::abs(pan) - minPanMoving) * translationSpeedFactor));

  if(forceBall)
  {
    for(Angle& pan : basePanAngles)
    {
      if(lookAtCloseObstacleWhenFollowingBall)
        pan = clipPanToNearObstacle(pan);
      pan = clipPanToBall(pan);
    }
  }

  if(panReached(basePanAngles[basePanAngleIndex]) && std::abs(basePanAngles[0] - basePanAngles[3]) > 10_deg)
    basePanAngleIndex = (basePanAngleIndex + 1) % basePanAngles.size();

  return basePanAngles[basePanAngleIndex];
}

Angle LibLookActiveProvider::calculateTilt(const bool forceBall, const bool onlyOwnBall) const
{
  if(ballPositionUnknown(onlyOwnBall))
  {
    return minTilt;
  }
  else if(forceBall)
  {
    Pose3f torsoMatrix = theTorsoMatrix;
    const Vector3f hip2Target = torsoMatrix
                                .rotateY(theCameraCalibration.bodyRotationCorrection.y())
                                .rotateX(theCameraCalibration.bodyRotationCorrection.x())
                                .inverse() * Vector3f(theBallPositionRelative.x(), theBallPositionRelative.y(), theBallSpecification.radius);
    Vector2a panTiltLowerCam, panTiltUpperCam;
    InverseKinematic::calcHeadJoints(hip2Target, pi_2, theRobotDimensions, CameraInfo::lower, panTiltLowerCam, theCameraCalibration);
    InverseKinematic::calcHeadJoints(hip2Target, pi_2, theRobotDimensions, CameraInfo::upper, panTiltUpperCam, theCameraCalibration);

    const Angle oldTilt = theJointRequest.stiffnessData.stiffnesses[Joints::headPitch] == 0 ? theJointAngles.angles[Joints::headPitch] : theJointRequest.angles[Joints::headPitch];
    Angle tilt = panTiltUpperCam.y();

    //if can not use upper camera && upper camera is not near || lower camera is near && can not easily use upper camera
    if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < ballPositionUnknownTimeoutLowerCamera &&
       ((tilt > theHeadLimits.getTiltBound(panTiltUpperCam.x()).max && std::abs(tilt - oldTilt) > cameraChoiceHysteresis) ||
        (std::abs(panTiltLowerCam.y() - oldTilt) < cameraChoiceHysteresis && tilt + cameraChoiceHysteresis > theHeadLimits.getTiltBound(panTiltUpperCam.x()).max)))
      tilt = panTiltLowerCam.y();
    return tilt;
  }
  else
  {
    return defaultTilt;
  }
}

Angle LibLookActiveProvider::calculateSpeed(const bool forceBall, const float, const float slowdownFactor) const
{
  // you cannot but want to look at the ball? then use maxSpeed
  // else defaultSpeed
  Angle speed = (forceBall && !ballInSight()) ? maxSpeed : defaultSpeed;

  float speedFactor = translationSpeedFactor * rotationSpeedFactor;
  speedFactor += 0.5f * std::log(std::abs(theJointAngles.angles[Joints::headYaw]) + 1.f);
  speedFactor = std::min(1.f, speedFactor * slowdownFactor);

  speed = minSpeed + speedFactor * (speed - minSpeed); // cannot be lower than minSpeed or higher than maxSpeed
  return speed;
}

bool LibLookActiveProvider::shouldLookAtBall() const
{
  if(!theGameState.isPlaying())
    return false;

  const auto striker = std::find_if(theAgentStates.agents.begin(), theAgentStates.agents.end(),
                                    [](const Agent& agent){return agent.role == ActiveRole::toRole(ActiveRole::playBall);});
  const Vector2f strikerBallPosition = striker != theAgentStates.agents.end() ? striker->ballPosition : theBallModel.estimate.position;
  const float distanceStrikerBallSquared = strikerBallPosition.squaredNorm();

  return distanceStrikerBallSquared < sqr(200.f)
         || distanceStrikerBallSquared > sqr(1500.f)
         || theBallPositionRelative.squaredNorm() < sqr(1000.f)
         || theBallSpeedRelative.squaredNorm() > sqr(100.f);
}

bool LibLookActiveProvider::panReached(const Angle targetPan) const
{
  const Angle currentPan = theJointAngles.angles[Joints::headYaw];
  return std::abs(currentPan - targetPan) < 5_deg;
}

Angle LibLookActiveProvider::clipPanToBall(const Angle pan) const
{
  Angle tolerance = theCameraInfo.openingAngleWidth / 6.f;
  const Angle ballAngle = theBallPositionRelative.angle();

  if(teamBallIsUsed)
  {
    const float distance = (theTeammatesBallModel.position - theRobotPose.translation).norm();
    tolerance += std::atan(teammatesBallModelError / distance);
  }
  return Rangea(ballAngle - tolerance, ballAngle + tolerance).limit(pan);
}

Angle LibLookActiveProvider::clipPanToNearObstacle(const Angle pan) const
{
  const Angle tolerance = theCameraInfo.openingAngleWidth / 3.f;

  const float maxSqrDistance = sqr(maxObstacleDistanceToBeLookedAt);
  float minSqrDistance = maxSqrDistance;
  Rangea obstacleClipRange(pan);

  for(const Obstacle& o : theObstacleModel.obstacles)
  {
    if(theFrameInfo.getTimeSince(o.lastSeen) < maxObstacleAgeToBeLookedAt)
    {
      const float sqrDistance = o.center.squaredNorm();

      if(sqrDistance < minSqrDistance)
      {
        const Angle obstacleAngle = o.center.angle();
        const Rangea obstacleRange(obstacleAngle - tolerance, obstacleAngle + tolerance);
        if(Rangea(theHeadLimits.minPan(), theHeadLimits.maxPan()).contains(obstacleRange))
        {
          minSqrDistance = sqrDistance;
          obstacleClipRange = obstacleRange;
        }
      }
    }
  }

  return obstacleClipRange.limit(pan);
}

bool LibLookActiveProvider::ballAnglesReachable() const
{
  const Angle tolerance = theCameraInfo.openingAngleWidth / 3.f;
  const Angle ballAngle = theBallPositionRelative.angle();

  // TODO: Check Shoulder?
  return Rangea(theHeadLimits.minPan(), theHeadLimits.maxPan()).contains(Rangea(ballAngle - tolerance, ballAngle + tolerance));
}

bool LibLookActiveProvider::ballInSight() const
{
  const Angle currentPan = theJointAngles.angles[Joints::headYaw];
  const Angle tolerance = theCameraInfo.openingAngleWidth / 3.f;
  const Angle ballAngle = theBallPositionRelative.angle();

  return Rangea(currentPan - tolerance, currentPan + tolerance).isInside(ballAngle);
}

bool LibLookActiveProvider::ballPositionUnknown(const bool onlyOwnBall) const
{
  const bool disappeared = theFrameInfo.getTimeSince(theBallModel.timeWhenDisappeared) > 100;
  const bool notSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > ballPositionUnknownTimeout;
  const bool globalNotSeen = !theTeammatesBallModel.isValid;

  return (disappeared && onlyOwnBall) || // Only our own ball counts and this one is lost
         (notSeen && globalNotSeen); // We and our team did not see the ball for a long time
}

void LibLookActiveProvider::calculateSpeedFactors()
{
  if(translationSpeedBuffer.full())
  {
    const float min = translationSpeedMinValue;
    const float max = translationSpeedMaxValue;

    translationSpeedFactor = translationSpeedBuffer.average() - min;
    translationSpeedFactor = std::min(max - min, std::max(0.f, translationSpeedFactor));
    translationSpeedFactor = translationSpeedFactor / (max - min);
    translationSpeedFactor = 1.f - translationSpeedFactor;
  }

  if(rotationSpeedBuffer.full())
  {
    const Angle min = rotationSpeedMinValue;
    const Angle max = rotationSpeedMaxValue;

    rotationSpeedFactor = rotationSpeedBuffer.average() - min;
    rotationSpeedFactor = std::min(max - min, std::max(0.f, rotationSpeedFactor));
    rotationSpeedFactor = rotationSpeedFactor / (max - min);
    rotationSpeedFactor = 1.f - rotationSpeedFactor;
  }
}
