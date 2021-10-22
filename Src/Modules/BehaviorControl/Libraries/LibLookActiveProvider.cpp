/**
 * @file LibLookActiveProvider.cpp
 * @author Andreas Stolpmann
 */

#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Math/Transformation.h"
#include "LibLookActiveProvider.h"
#include "Tools/Math/Geometry.h"

MAKE_MODULE(LibLookActiveProvider, behaviorControl);

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
  libLookActive.calculateHeadTarget = [this](const bool withBall, const bool ignoreBall, const bool onlyOwnBall, const bool fixTilt) -> HeadTarget
  {
    return calculateHeadTarget(withBall, ignoreBall, onlyOwnBall, fixTilt);
  };
}

HeadTarget LibLookActiveProvider::calculateHeadTarget(const bool withBall, const bool ignoreBall, const bool onlyOwnBall, const bool fixTilt)
{
  theBallPositionRelative = theBallModel.estimate.position;
  theBallSpeedRelative = theBallModel.estimate.velocity;
  if(!onlyOwnBall
     && (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > ballPositionUnknownTimeout || theFrameInfo.getTimeSince(theBallModel.timeWhenDisappeared) > 100)
     && theTeamBallModel.timeWhenLastSeen > theBallModel.timeWhenLastSeen)
  {
    theBallPositionRelative = Transformation::fieldToRobot(theRobotPose, theTeamBallModel.position);
    theBallSpeedRelative = theTeamBallModel.velocity.rotated(-theRobotPose.rotation);
  }

  calculateSpeedFactors();
  const bool forceBall = !ignoreBall && !ballPositionUnknown(onlyOwnBall) && (withBall || shouldLookAtBall()) && ballAnglesReachable();

  const Angle pan = calculatePan(forceBall);
  const Angle tilt = fixTilt ? Angle(0.38f) : calculateTilt(forceBall, onlyOwnBall);
  const Angle speed = calculateSpeed(forceBall, pan);

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
    Vector3f hip2Target = theTorsoMatrix.inverse() * Vector3f(theBallPositionRelative.x(), theBallPositionRelative.y(), theBallSpecification.radius);
    Vector2a panTiltLowerCam, panTiltUpperCam;
    InverseKinematic::calcHeadJoints(hip2Target, pi_2, theRobotDimensions, CameraInfo::lower, panTiltLowerCam, theCameraCalibration);
    InverseKinematic::calcHeadJoints(hip2Target, pi_2, theRobotDimensions, CameraInfo::upper, panTiltUpperCam, theCameraCalibration);

    Angle tilt = panTiltUpperCam.y() + (panTiltLowerCam.y() - panTiltUpperCam.y());
    if(defaultTilt < tilt)
      tilt = std::min(minTilt, tilt);
    else
      tilt = defaultTilt;
    return tilt;
  }
  else
  {
    return defaultTilt;
  }
}

Angle LibLookActiveProvider::calculateSpeed(const bool forceBall, const float) const
{
  Angle speed = (forceBall && !ballInSight()) ? maxSpeed : defaultSpeed;

  float speedFactor = translationSpeedFactor * rotationSpeedFactor;
  speedFactor += 0.5f * std::log(std::abs(theJointAngles.angles[Joints::headYaw]) + 1.f);
  speedFactor = std::min(1.f, speedFactor);

  speed = minSpeed + speedFactor * (speed - minSpeed);
  return speed;
}

bool LibLookActiveProvider::shouldLookAtBall() const
{
  if(theGameInfo.state != STATE_PLAYING)
    return false;

  const float distanceStrikerBallSquared = theLibTeam.getBallPosition(theLibTeam.strikerPlayerNumber).squaredNorm();

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
  const Angle tolerance = theCameraInfo.openingAngleWidth / 3.f;
  const Angle ballAngle = theBallPositionRelative.angle();

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
  const bool disappeared = theFrameInfo.getTimeSince(theBallModel.timeWhenDisappeared) > 100
                           && theTeamBallModel.timeWhenLastSeen < theBallModel.timeWhenDisappeared;
  const bool notSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > ballPositionUnknownTimeout;
  const bool globalNotSeen = theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastSeen) > ballPositionUnknownTimeout;

  return disappeared || (notSeen && (onlyOwnBall || globalNotSeen));
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
