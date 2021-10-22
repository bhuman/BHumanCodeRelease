/**
 * @file SimulatedRobot2D.cpp
 *
 * This file implements the interface to a robot with the 2D simulation core.
 *
 * @author Arne Hasselbring
 */

#include "SimulatedRobot2D.h"
#include "Controller/RoboCupCtrl.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Configuration/CameraResolutionRequest.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Streams/InStreams.h"
#include <SimRobotCore2D.h>

SimulatedRobot2D::SimulatedRobot2D(SimRobot::Object* robot) :
  SimulatedRobot(robot)
{
  {
    InMapFile stream("kickInfo.cfg");
    ASSERT(stream.exists());
    stream >> kickInfo;

    FOREACH_ENUM(KickInfo::KickType, kickType)
    {
      kicks[kickType].type = kickType;
      if(kickInfo[kickType].motion == MotionPhase::kick)
      {
        kicks[kickType].impactT = 0.65f;
        kicks[kickType].duration = 1.65f;
      }
      else
      {
        kicks[kickType].impactT = baseWalkPeriod * 0.5f;
        kicks[kickType].steps.emplace_back(Angle::normalize(-(kickInfo[kickType].postRotationOffset + kickInfo[kickType].rotationOffset)), 0.f, 0.f);
      }
      if(kickType == KickInfo::forwardFastLeft || kickType == KickInfo::forwardFastRight)
        kicks[kickType].distanceRange = Rangef(1000.f, 6000.f);
      else
        kicks[kickType].distanceRange = kickInfo[kickType].range;
      kicks[kickType].distanceDeviation = 0.07f;
      kicks[kickType].direction = -kickInfo[kickType].rotationOffset;
      kicks[kickType].directionDeviation = 7_deg;
    }
  }

  {
    InMapFile stream("ballSpecification.cfg");
    ASSERT(stream.exists());
    stream >> ballSpecification;
  }

  // load camera parameters
  CameraIntrinsics cameraIntrinsics;
  {
    InMapFile stream("cameraIntrinsics.cfg");
    ASSERT(stream.exists());
    stream >> cameraIntrinsics;
  }

  CameraResolutionRequest cameraResolutionRequest;
  {
    InMapFile stream("cameraResolution.cfg");
    ASSERT(stream.exists());
    stream >> cameraResolutionRequest;
  }

  // build cameraInfo
  FOREACH_ENUM(CameraInfo::Camera, camera)
  {
    cameraInfos[camera].camera = camera;

    switch(cameraResolutionRequest.resolutions[camera])
    {
      case CameraResolutionRequest::w320h240:
        cameraInfos[camera].width = 320;
        cameraInfos[camera].height = 240;
        break;
      case CameraResolutionRequest::w640h480:
        cameraInfos[camera].width = 640;
        cameraInfos[camera].height = 480;
        break;
      case CameraResolutionRequest::w1280h960:
        cameraInfos[camera].width = 1280;
        cameraInfos[camera].height = 960;
        break;
      default:
        ASSERT(false);
        break;
    }

    // set opening angle
    cameraInfos[camera].openingAngleWidth = cameraIntrinsics.cameras[camera].openingAngleWidth;
    cameraInfos[camera].openingAngleHeight = cameraIntrinsics.cameras[camera].openingAngleHeight;
    // set optical center
    cameraInfos[camera].opticalCenter.x() = cameraIntrinsics.cameras[camera].opticalCenter.x() * cameraInfos[camera].width;
    cameraInfos[camera].opticalCenter.y() = cameraIntrinsics.cameras[camera].opticalCenter.y() * cameraInfos[camera].height;
    // update focal length
    cameraInfos[camera].updateFocalLength();
  }

  // Get some geometry that belongs to the robot in order to create collisions with it.
  const int robotChildren = RoboCupCtrl::application->getObjectChildCount(*robot);
  for(int i = 0; i < robotChildren; ++i)
  {
    auto* robotGeom = RoboCupCtrl::application->getObjectChild(*robot, i);
    if(robotGeom->getKind() == SimRobotCore2D::geometry)
    {
      if(!robotGeometry)
        robotGeometry = reinterpret_cast<SimRobotCore2D::Geometry*>(robotGeom);

      reinterpret_cast<SimRobotCore2D::Geometry*>(robotGeom)->registerCollisionCallback(*this);
    }
  }
  ASSERT(robotGeometry);
}

void SimulatedRobot2D::getRobotPose(Pose2f& robotPose) const
{
  getPose2f(robot, robotPose);
  if(firstTeam)
    robotPose = Pose2f(pi) + robotPose;
}

void SimulatedRobot2D::getImage(CameraImage& cameraImage, CameraInfo& cameraInfo)
{
  cameraInfo = cameraInfos[currentCamera];
  cameraImage.setResolution(0, 0);
  cameraImage.timestamp = Time::getCurrentSystemTime();
}

void SimulatedRobot2D::getCameraInfo(CameraInfo& cameraInfo)
{
  cameraInfo = cameraInfos[currentCamera];
}

void SimulatedRobot2D::setJointCalibration(const JointCalibration&)
{}

void SimulatedRobot2D::getAndSetJointData(const JointRequest&, JointSensorData& jointSensorData) const
{
  jointSensorData.timestamp = Time::getCurrentSystemTime();
}

void SimulatedRobot2D::setJointRequest(const JointRequest&) const
{}

void SimulatedRobot2D::toggleCamera()
{
  currentCamera = currentCamera == CameraInfo::upper ? CameraInfo::lower : CameraInfo::upper;
}

void SimulatedRobot2D::getSensorData(FsrSensorData&, InertialSensorData&)
{}

void SimulatedRobot2D::getAndSetMotionData(const MotionRequest& motionRequest, MotionInfo& motionInfo)
{
  // 0. Displace the robot if it's locked with other robots.
  // 0.1 Update the collision ring buffers.
  for(auto& pair : collisionBuffer)
  {
    if(collisionsThisFrame.count(pair.first))
    {
      pair.second.push_front(1.f);
      collisionsThisFrame.erase(pair.first);
    }
    else
      pair.second.push_front(0.f);
  }
  for(SimRobotCore2D::Body* body : collisionsThisFrame)
  {
    auto& buffer = collisionBuffer[body];
    while(!buffer.full())
      buffer.push_front(0.f);
    buffer.push_front(1.f);
  }
  collisionsThisFrame.clear();
  // 0.2 Move the robot if it has collided too often.
  for(auto& pair : collisionBuffer)
  {
    if(pair.second.front() < 1.f || pair.second.average() <= 0.4f || Random::uniform(0.f, 0.6f) > pair.second.average() - 0.4f)
      continue;

    // Shift it away from the other body by a random amount.
    const float* otherPosition = pair.first->getPosition();
    const float* robotPosition = static_cast<SimRobotCore2D::Body*>(robot)->getPosition();
    const Vector2f target = Vector2f(robotPosition[0], robotPosition[1]) -
                            (Vector2f(otherPosition[0], otherPosition[1]) - Vector2f(robotPosition[0], robotPosition[1])).normalized(Random::uniform(0.07f, 0.13f));
    static_cast<SimRobotCore2D::Body*>(robot)->move(target.data());
  }

  // 1. Advance time.
  const float dt = RoboCupCtrl::controller->simStepLength * 0.001f;
  currentPhase.t += dt;
  // 2. Did the phase end?
  bool isDone = currentPhase.t >= currentPhase.duration;
  if(isDone)
  {
    // 2.a Start new motion from motion request.
    currentPhase.t = 0.f;
    currentPhase.performedAction = false;
    if(currentPhase.kick)
    {
      motionInfo.lastKickTimestamp = Time::getCurrentSystemTime();
      motionInfo.lastKickType = currentPhase.kick->type;
      currentPhase.kick = nullptr;
    }
    switch(motionRequest.motion)
    {
      case MotionRequest::playDead:
        requestPlayDead();
        break;
      case MotionRequest::stand:
        requestStand(motionRequest.standHigh);
        break;
      case MotionRequest::walkAtAbsoluteSpeed:
        requestWalkAtAbsoluteSpeed(motionRequest.walkSpeed);
        break;
      case MotionRequest::walkAtRelativeSpeed:
        requestWalkAtRelativeSpeed(motionRequest.walkSpeed);
        break;
      case MotionRequest::walkToPose:
        requestWalkToPose(motionRequest.walkTarget, motionRequest.walkSpeed, motionRequest.keepTargetRotation, motionRequest.obstacleAvoidance);
        break;
      case MotionRequest::walkToBallAndKick:
        requestWalkToBallAndKick(motionRequest.ballEstimate, motionRequest.targetDirection, motionRequest.kickType, motionRequest.kickPower,
                                 motionRequest.alignPrecisely, motionRequest.walkSpeed, motionRequest.obstacleAvoidance);
        break;
      case MotionRequest::dribble:
        requestDribble(motionRequest.ballEstimate, motionRequest.targetDirection, motionRequest.walkSpeed, motionRequest.obstacleAvoidance);
        break;
      case MotionRequest::getUp:
        requestGetUp();
        break;
      case MotionRequest::keyframeMotion:
        requestKeyframeMotion(motionRequest.keyframeMotionRequest);
        break;
      default:
        FAIL("Unknown motion type.");
    }
  }
  // 3. Execute motion (and fill motion info).
  switch(currentPhase.motion)
  {
    case MotionPhase::playDead:
      executePlayDead(motionInfo);
      break;
    case MotionPhase::stand:
      executeStand(motionInfo);
      break;
    case MotionPhase::walk:
      executeWalk(motionInfo);
      break;
    case MotionPhase::kick:
      executeKick(motionInfo);
      break;
    case MotionPhase::fall:
      executeFall(motionInfo);
      break;
    case MotionPhase::getUp:
      executeGetUp(motionInfo);
      break;
    case MotionPhase::keyframeMotion:
      executeKeyframeMotion(motionInfo);
      break;
    default:
      FAIL("Unknown motion type.");
  }
}

void SimulatedRobot2D::moveRobot(const Vector3f& pos, const Vector3f& rot, bool changeRotation)
{
  const Vector3f position = pos * 0.001f;
  if(changeRotation)
    static_cast<SimRobotCore2D::Body*>(robot)->move(position.data(), rot.z());
  else
    static_cast<SimRobotCore2D::Body*>(robot)->move(position.data());
}

void SimulatedRobot2D::enablePhysics(bool enable)
{
  static_cast<SimRobotCore2D::Body*>(robot)->enablePhysics(enable);
}

bool SimulatedRobot2D::getPose2f(const SimRobot::Object* obj, Pose2f& pose) const
{
  static_cast<const SimRobotCore2D::Body*>(obj)->getPose(pose.translation.data(), reinterpret_cast<float*>(&pose.rotation));
  pose.translation *= 1000.f;
  return true;
}

void SimulatedRobot2D::getPose3f(const SimRobot::Object* obj, Pose3f& pose) const
{
  float rotation;
  static_cast<const SimRobotCore2D::Body*>(obj)->getPose(pose.translation.data(), &rotation);
  pose.translation.z() = 0.f;
  pose.translation *= 1000.f;
  pose.rotation = RotationMatrix::aroundZ(rotation);
}

void SimulatedRobot2D::collided(SimRobotCore2D::Geometry&, SimRobotCore2D::Geometry& geom2)
{
  SimRobotCore2D::Body* body2 = geom2.getParentBody();
  if(!body2 || body2 == ball)
    return;

  collisionsThisFrame.insert(body2->getRootBody());
}

void SimulatedRobot2D::requestPlayDead()
{
  currentPhase.motion = MotionPhase::playDead;
  currentPhase.duration = 0.f;
}

void SimulatedRobot2D::requestStand(bool standHigh)
{
  currentPhase.motion = MotionPhase::stand;
  currentPhase.duration = standHigh ? 0.7f : 0.f;
}

void SimulatedRobot2D::requestWalkAtAbsoluteSpeed(const Pose2f& walkSpeed)
{
  currentPhase.isLeftPhase = isNextLeftPhase(walkSpeed.translation.y() != 0.f ? (walkSpeed.translation.y() > 0.f) : (walkSpeed.rotation > 0.f));
  currentPhase.motion = MotionPhase::walk;
  currentPhase.duration = baseWalkPeriod;
  currentPhase.step = walkSpeed;
  currentPhase.step.rotation = getRotationRange(currentPhase.isLeftPhase, Pose2f(1.f, 1.f, 1.f)).clamped(walkSpeed.rotation);
  {
    Vector2f backRight, frontLeft;
    getTranslationRectangle(currentPhase.step.rotation, Pose2f(1.f, 1.f, 1.f), backRight, frontLeft);
    if(Geometry::isPointInsideRectangle(backRight, frontLeft, walkSpeed.translation))
      currentPhase.step.translation = walkSpeed.translation;
    else
    {
      Vector2f p1, p2;
      VERIFY(Geometry::getIntersectionPointsOfLineAndRectangle(backRight, frontLeft, Geometry::Line(Vector2f(0.f, 0.f), walkSpeed.translation.normalized()), p1, p2));
      currentPhase.step.translation = p2;
    }
    if(currentPhase.isLeftPhase == (currentPhase.step.translation.y() < 0.f))
      currentPhase.step.translation.y() = 0.f;
  }
}

void SimulatedRobot2D::requestWalkAtRelativeSpeed(const Pose2f& walkSpeed)
{
  Pose2f absoluteSpeed(walkSpeed.rotation * maxSpeed.rotation, walkSpeed.translation.array() * maxSpeed.translation.array());
  if(walkSpeed.translation.x() < 0.f)
    absoluteSpeed.translation.x() = walkSpeed.translation.x() * maxSpeedBackwards;
  requestWalkAtAbsoluteSpeed(absoluteSpeed);
}

void SimulatedRobot2D::requestWalkToPose(const Pose2f& walkTarget, const Pose2f& walkSpeed, bool keepTargetRotation, const MotionRequest::ObstacleAvoidance& obstacleAvoidance)
{
  const Angle controlAheadAngle = 0.3f;
  const float startTurningBeforeCircleDistance = 200.f;

  bool interpolateRotation = !keepTargetRotation;
  Pose2f modTarget = walkTarget;
  if(!obstacleAvoidance.path.empty())
  {
    const auto& segment = obstacleAvoidance.path.front();
    const float sign = segment.clockwise ? 1.f : -1.f;
    Vector2f toPoint = segment.obstacle.center;
    if(segment.obstacle.radius != 0.f)
    {
      const Angle tangentOffset = std::asin(std::min(1.f, segment.obstacle.radius / segment.obstacle.center.norm()));
      toPoint.rotate(sign * tangentOffset);
      toPoint *= std::cos(tangentOffset);
      interpolateRotation = false;
    }
    if(toPoint.squaredNorm() <= sqr(startTurningBeforeCircleDistance))
    {
      const Vector2f offset = toPoint - segment.obstacle.center;
      const Pose2f controlPose(Angle::normalize(offset.angle() + (segment.clockwise ? -pi_2 - controlAheadAngle : pi_2 + controlAheadAngle)),
                               Pose2f(segment.clockwise ? -controlAheadAngle : controlAheadAngle, segment.obstacle.center) * (offset / std::cos(controlAheadAngle)));

      modTarget.translation = controlPose.translation;
      if(!keepTargetRotation)
        modTarget.rotation = controlPose.rotation;
    }
    else
    {
      modTarget.translation = toPoint;
      if(!keepTargetRotation)
        modTarget.rotation = toPoint.angle();
    }
  }

  if(interpolateRotation)
  {
    if(walkTarget.translation.norm() > 600.f - std::abs(walkTarget.rotation) / pi * 400.f)
    {
      modTarget.rotation = walkTarget.translation.angle();
    }
    else if(walkTarget.translation.norm() > 100.f)
    {
      const float factor = (walkTarget.translation.norm() - 100.f) / 500.f;
      modTarget.rotation = (factor * walkTarget.translation.normalized() + Vector2f::polar(1.f - factor, walkTarget.rotation)).angle();
    }
  }

  currentPhase.isLeftPhase = isNextLeftPhase(walkTarget.translation.y() != 0.f ? (walkTarget.translation.y() > 0.f) : (walkTarget.rotation > 0.f));
  currentPhase.motion = MotionPhase::walk;
  currentPhase.duration = baseWalkPeriod;
  currentPhase.step.rotation = getRotationRange(currentPhase.isLeftPhase, walkSpeed).clamped(keepTargetRotation ? walkTarget.rotation : modTarget.rotation);
  {
    Vector2f backRight, frontLeft;
    getTranslationRectangle(currentPhase.step.rotation, walkSpeed, backRight, frontLeft);
    if(Geometry::isPointInsideRectangle(backRight, frontLeft, walkTarget.translation))
      currentPhase.step.translation = walkTarget.translation;
    else
    {
      Vector2f p1, p2;
      const Vector2f targetDirectionWithAvoidance = obstacleAvoidance.avoidance + modTarget.translation.normalized(std::max(0.f, 1.f - obstacleAvoidance.avoidance.norm()));
      VERIFY(Geometry::getIntersectionPointsOfLineAndRectangle(backRight, frontLeft, Geometry::Line(Vector2f(0.f, 0.f), targetDirectionWithAvoidance), p1, p2));
      currentPhase.step.translation = p2;
    }
    if(currentPhase.isLeftPhase == (currentPhase.step.translation.y() < 0.f))
      currentPhase.step.translation.y() = 0.f;
  }
}

void SimulatedRobot2D::requestWalkToBall(const Pose2f& kickPose, const Vector2f& ballPosition, const Pose2f& walkSpeed, const MotionRequest::ObstacleAvoidance& obstacleAvoidance)
{
  if(!obstacleAvoidance.path.empty() || std::abs(Angle::normalize((ballPosition - kickPose.translation).angle() - kickPose.translation.angle())) < 90_deg)
  {
    requestWalkToPose(kickPose, walkSpeed, false, obstacleAvoidance);
    return;
  }

  const Vector2f targetInBall = kickPose.translation - ballPosition;
  const float ballCircleRadius = std::max(targetInBall.norm(), ballSpecification.radius + 110.f + 50.f);
  const Angle tangentOffset = std::asin(std::min(1.f, ballCircleRadius / ballPosition.norm()));
  const Vector2f tangentPointUnscaledCW = ballPosition.rotated(tangentOffset);
  const Vector2f tangentPointUnscaledCCW = ballPosition.rotated(-tangentOffset);
  const float cosTangentOffset = std::cos(tangentOffset);
  const Vector2f tangentPointCW = tangentPointUnscaledCW * cosTangentOffset;
  const Vector2f tangentPointCCW = tangentPointUnscaledCCW * cosTangentOffset;

  Angle arcAngleCW = (tangentPointCW - ballPosition).angle() - targetInBall.angle();
  if(arcAngleCW < 0.f)
    arcAngleCW += pi2;
  Angle arcAngleCCW = targetInBall.angle() - (tangentPointCCW - ballPosition).angle();
  if(arcAngleCCW < 0.f)
    arcAngleCCW += pi2;

  const bool cw = arcAngleCW < arcAngleCCW;
  const Vector2f& selectedTangentPoint = cw ? tangentPointCW : tangentPointCCW;

  const Angle angleOffset = (kickPose.inverse() * ballPosition).angle();
  if(std::abs(Angle::normalize(angleOffset - ballPosition.angle())) > 50_deg || ballPosition.squaredNorm() > sqr(ballCircleRadius + 100.f))
  {
    return requestWalkToPose(Pose2f(Angle::normalize((ballPosition - selectedTangentPoint).angle() - angleOffset), selectedTangentPoint), walkSpeed,
                             false, obstacleAvoidance);
  }
  const Vector2f tangentPointUnscaled = cw ? tangentPointUnscaledCW : tangentPointUnscaledCCW;
  const Pose2f modTarget(Angle::normalize((ballPosition - tangentPointUnscaled.normalized(std::min(100.f, kickPose.translation.norm()))).angle() - angleOffset), tangentPointUnscaled);

  currentPhase.isLeftPhase = isNextLeftPhase(modTarget.translation.y() != 0.f ? (modTarget.translation.y() > 0.f) : (modTarget.rotation > 0.f));
  currentPhase.motion = MotionPhase::walk;
  currentPhase.duration = baseWalkPeriod;
  currentPhase.step.rotation = getRotationRange(currentPhase.isLeftPhase, walkSpeed).clamped(modTarget.rotation);
  {
    Vector2f backRight, frontLeft;
    getTranslationRectangle(currentPhase.step.rotation, walkSpeed, backRight, frontLeft);
    if(Geometry::isPointInsideRectangle(backRight, frontLeft, kickPose.translation))
      currentPhase.step.translation = kickPose.translation;
    else
    {
      Vector2f p1, p2;
      const Vector2f targetDirectionWithAvoidance = obstacleAvoidance.avoidance + modTarget.translation.normalized(std::max(0.f, 1.f - obstacleAvoidance.avoidance.norm()));
      VERIFY(Geometry::getIntersectionPointsOfLineAndRectangle(backRight, frontLeft, Geometry::Line(Vector2f(0.f, 0.f), targetDirectionWithAvoidance), p1, p2));
      currentPhase.step.translation = p2;
    }
    if(currentPhase.isLeftPhase == (currentPhase.step.translation.y() < 0.f))
      currentPhase.step.translation.y() = 0.f;
  }
}

void SimulatedRobot2D::requestWalkToBallAndKick(const BallState& ballEstimate, Angle targetDirection, KickInfo::KickType kickType, float kickPower, bool alignPrecisely, const Pose2f& walkSpeed, const MotionRequest::ObstacleAvoidance& obstacleAvoidance)
{
  const Vector2f& ballPosition = ballEstimate.position;
  bool isInPositionForKick = false;
  const Angle directionThreshold = alignPrecisely ? 2_deg : 3_deg;
  switch(kickType)
  {
    case KickInfo::forwardFastLeft:
    case KickInfo::forwardFastLeftLong:
    case KickInfo::walkForwardsLeft:
    case KickInfo::walkForwardsLeftLong:
      isInPositionForKick = ballPosition.y() > 30.f && ballPosition.y() < 60.f && ballPosition.x() < (kickType == KickInfo::walkForwardsLeftLong ? 270.f : (kickType == KickInfo::forwardFastLeftLong ? 270.f : 200.f)) && ballPosition.x() > 100.f;
      isInPositionForKick &= std::abs(targetDirection) < directionThreshold;
      break;
    case KickInfo::forwardFastRight:
    case KickInfo::forwardFastRightLong:
    case KickInfo::walkForwardsRight:
    case KickInfo::walkForwardsRightLong:
      isInPositionForKick = -ballPosition.y() > 30.f && -ballPosition.y() < 60.f && ballPosition.x() < (kickType == KickInfo::walkForwardsRightLong ? 270.f : (kickType == KickInfo::forwardFastRightLong ? 270.f : 200.f)) && ballPosition.x() > 100.f;
      isInPositionForKick &= std::abs(targetDirection) < directionThreshold;
      break;
    case KickInfo::walkSidewardsLeftFootToLeft:
      isInPositionForKick = ballPosition.y() > 30.f && ballPosition.y() < 180.f && ballPosition.x() > 0.f && ballPosition.x() < 100.f;
      isInPositionForKick &= std::abs(targetDirection - 90_deg) < directionThreshold;
      break;
    case KickInfo::walkSidewardsRightFootToRight:
      isInPositionForKick = -ballPosition.y() > 30.f && -ballPosition.y() < 180.f && ballPosition.x() > 0.f && ballPosition.x() < 100.f;
      isInPositionForKick &= std::abs(targetDirection + 90_deg) < directionThreshold;
      break;
    case KickInfo::walkTurnRightFootToLeft:
    {
      const Vector2f ballPositionAfterPreStep = Pose2f(45_deg, 30.f, 20.f).inverse() * ballPosition;
      isInPositionForKick = -ballPositionAfterPreStep.y() > 30.f && -ballPositionAfterPreStep.y() < 70.f && ballPositionAfterPreStep.x() > 50.f && ballPositionAfterPreStep.x() < 170.f;
      isInPositionForKick &= std::abs(targetDirection - 45_deg) < directionThreshold;
      break;
    }
    case KickInfo::walkTurnLeftFootToRight:
    {
      const Vector2f ballPositionAfterPreStep = Pose2f(-45_deg, 30.f, -20.f).inverse() * ballPosition;
      isInPositionForKick = ballPositionAfterPreStep.y() > 30.f && ballPositionAfterPreStep.y() < 70.f && ballPositionAfterPreStep.x() > 50.f && ballPositionAfterPreStep.x() < 170.f;
      isInPositionForKick &= std::abs(targetDirection + 45_deg) < directionThreshold;
      break;
    }
    default:
      FAIL("Unknown kick type.");
  }

  if(isInPositionForKick)
  {
    currentPhase.kick = &kicks[kickType];
    currentPhase.kickPower = kickPower;
    if(!kicks[kickType].steps.empty())
    {
      currentPhase.motion = MotionPhase::walk;
      currentPhase.duration = baseWalkPeriod;
      currentPhase.step = kicks[kickType].steps[0];
    }
    else
    {
      currentPhase.motion = MotionPhase::kick;
      currentPhase.duration = kicks[kickType].duration;
    }
  }
  else
  {
    const Vector2f ballEndPosition = BallPhysics::getEndPosition(ballEstimate.position, ballEstimate.velocity, ballSpecification.friction);
    const Pose2f kickPose = Pose2f(targetDirection, ballEndPosition).rotate(kickInfo[kickType].rotationOffset).translate(kickInfo[kickType].ballOffset);
    requestWalkToBall(kickPose, ballEndPosition, walkSpeed, obstacleAvoidance);
  }
}

void SimulatedRobot2D::requestDribble(const BallState& ballEstimate, Angle targetDirection, const Pose2f& walkSpeed, const MotionRequest::ObstacleAvoidance& obstacleAvoidance)
{
  const Vector2f ballEndPosition = BallPhysics::getEndPosition(ballEstimate.position, ballEstimate.velocity, ballSpecification.friction);
  const Pose2f kickPose = Pose2f(targetDirection, ballEndPosition).translate(-170.f, 0.f);
  if(ballEstimate.position.x() > 50.f && ballEstimate.position.x() < 200.f && std::abs(ballEstimate.position.y()) < 70.f && std::abs(targetDirection) < 5_deg)
  {
    currentPhase.kick = &kicks[ballEstimate.position.y() > 0.f ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight];
    currentPhase.kickPower = 0.2f;
    currentPhase.motion = MotionPhase::walk;
    currentPhase.duration = baseWalkPeriod;
    currentPhase.step = currentPhase.kick->steps[0];
  }
  else
    requestWalkToBall(kickPose, ballEndPosition, walkSpeed, obstacleAvoidance);
}

void SimulatedRobot2D::requestGetUp()
{
  currentPhase.motion = MotionPhase::getUp;
  currentPhase.duration = 0.f;
}

void SimulatedRobot2D::requestKeyframeMotion(const KeyframeMotionRequest& keyframeMotionRequest)
{
  currentPhase.motion = MotionPhase::keyframeMotion;
  currentPhase.keyframeMotionRequest = keyframeMotionRequest;
  switch(keyframeMotionRequest.keyframeMotion)
  {
    case KeyframeMotionRequest::keeperJumpLeft:
      currentPhase.duration = 3.f;
      break;
    case KeyframeMotionRequest::genuflectFromSitting:
    case KeyframeMotionRequest::genuflectStand:
    case KeyframeMotionRequest::genuflectStandDefender:
      currentPhase.duration = 2.f;
      break;
    default:
      currentPhase.duration = 0.f;
      break;
  }
}

void SimulatedRobot2D::executePlayDead(MotionInfo& motionInfo)
{
  static_cast<SimRobotCore2D::Body*>(robot)->setVelocity(zero, 0.f);
  motionInfo.executedPhase = MotionPhase::playDead;
  motionInfo.isMotionStable = false;
}

void SimulatedRobot2D::executeStand(MotionInfo& motionInfo)
{
  static_cast<SimRobotCore2D::Body*>(robot)->setVelocity(zero, 0.f);
  motionInfo.executedPhase = MotionPhase::stand;
  motionInfo.isMotionStable = true;
}

void SimulatedRobot2D::executeWalk(MotionInfo& motionInfo)
{
  const float dt = RoboCupCtrl::controller->simStepLength * 0.001f;
  const float f0 = currentPhase.t / currentPhase.duration;
  const float f1 = (currentPhase.t + dt) / currentPhase.duration;
  const Pose2f pose0(f0 * currentPhase.step.rotation, f0 * currentPhase.step.translation);
  const Pose2f pose1(f1 * currentPhase.step.rotation, f1 * currentPhase.step.translation);
  Pose2f diff = pose0.inverse() * pose1;
  diff.translation /= dt;
  diff.rotation /= dt;

  float rotation = 0.f;
  static_cast<SimRobotCore2D::Body*>(robot)->getPose(nullptr, &rotation);
  const Vector2f linearSpeed = diff.translation.rotated(rotation) * 0.001f;
  static_cast<SimRobotCore2D::Body*>(robot)->setVelocity(linearSpeed.data(), diff.rotation);

  if(currentPhase.kick)
  {
    if(currentPhase.t >= currentPhase.kick->impactT && !currentPhase.performedAction)
    {
      currentPhase.performedAction = true;

      const float randomRange = Random::triangular(1.f, currentPhase.kick->distanceDeviation);
      const float randomDirection = Random::normal<float>(0.f, currentPhase.kick->directionDeviation);
      const Rangef& kickRangeRange = currentPhase.kick->distanceRange;

      const Vector2f velocity = Vector2f::polar(BallPhysics::velocityForDistance(kickRangeRange.min + (kickRangeRange.max - kickRangeRange.min) * currentPhase.kickPower, ballSpecification.friction) * 0.001f * randomRange,
                                                randomDirection + rotation + currentPhase.kick->direction - f0 * currentPhase.step.rotation);
      // Fake a collision with the ball, such that the GameController knows about it.
      static_cast<SimRobotCore2D::CollisionCallback*>(RoboCupCtrl::controller)->collided(*static_cast<SimRobotCore2D::Geometry*>(RoboCupCtrl::controller->ballGeometry), *robotGeometry);
      static_cast<SimRobotCore2D::Body*>(ball)->setVelocity(velocity.data());
    }
  }

  motionInfo.executedPhase = MotionPhase::walk;
  motionInfo.isMotionStable = true;
  motionInfo.isWalkPhaseInWalkKick = currentPhase.kick;
  motionInfo.speed = currentPhase.step;
  motionInfo.speed.translation /= currentPhase.duration;
  motionInfo.speed.rotation /= currentPhase.duration;
}

void SimulatedRobot2D::executeKick(MotionInfo& motionInfo)
{
  ASSERT(currentPhase.kick);
  static_cast<SimRobotCore2D::Body*>(robot)->setVelocity(zero, 0.f);

  if(currentPhase.t >= currentPhase.kick->impactT && !currentPhase.performedAction)
  {
    currentPhase.performedAction = true;
    float rotation = 0.f;
    static_cast<SimRobotCore2D::Body*>(robot)->getPose(nullptr, &rotation);

    const float randomRange = Random::triangular(1.f, currentPhase.kick->distanceDeviation);
    const float randomDirection = Random::normal<float>(0.f, currentPhase.kick->directionDeviation);
    const Rangef& kickRangeRange = currentPhase.kick->distanceRange;

    const Vector2f velocity = Vector2f::polar(BallPhysics::velocityForDistance(kickRangeRange.min + (kickRangeRange.max - kickRangeRange.min) * currentPhase.kickPower, ballSpecification.friction) * 0.001f * randomRange,
                                              randomDirection + rotation + currentPhase.kick->direction);
    // Fake a collision with the ball, such that the GameController knows about it.
    static_cast<SimRobotCore2D::CollisionCallback*>(RoboCupCtrl::controller)->collided(*static_cast<SimRobotCore2D::Geometry*>(RoboCupCtrl::controller->ballGeometry), *robotGeometry);
    static_cast<SimRobotCore2D::Body*>(ball)->setVelocity(velocity.data());
  }

  motionInfo.executedPhase = MotionPhase::kick;
  motionInfo.isMotionStable = true;
}

void SimulatedRobot2D::executeFall(MotionInfo& motionInfo)
{
  static_cast<SimRobotCore2D::Body*>(robot)->setVelocity(zero, 0.f);
  motionInfo.executedPhase = MotionPhase::fall;
  motionInfo.isMotionStable = false;
}

void SimulatedRobot2D::executeGetUp(MotionInfo& motionInfo)
{
  static_cast<SimRobotCore2D::Body*>(robot)->setVelocity(zero, 0.f);
  motionInfo.executedPhase = MotionPhase::getUp;
  motionInfo.isMotionStable = false;
  motionInfo.getUpTryCounter = 0;
}

void SimulatedRobot2D::executeKeyframeMotion(MotionInfo& motionInfo)
{
  static_cast<SimRobotCore2D::Body*>(robot)->setVelocity(zero, 0.f);
  if(currentPhase.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::keeperJumpLeft)
  {
    if(currentPhase.t >= 0.5f && !currentPhase.performedAction)
    {
      currentPhase.performedAction = true;
      const Vector2f offset(0.f, currentPhase.keyframeMotionRequest.mirror ? -.3f : .3f);
      Pose2f robotPose;
      static_cast<SimRobotCore2D::Body*>(robot)->getPose(robotPose.translation.data(), &static_cast<float&>(robotPose.rotation));
      const Vector2f target = robotPose * offset;
      static_cast<SimRobotCore2D::Body*>(robot)->move(target.data());
    }
  }
  motionInfo.executedPhase = MotionPhase::keyframeMotion;
  motionInfo.isMotionStable = false;
  motionInfo.executedKeyframeMotion = currentPhase.keyframeMotionRequest;
}

bool SimulatedRobot2D::isNextLeftPhase(bool shouldBeLeft) const
{
  if(currentPhase.motion != MotionPhase::walk)
    return shouldBeLeft;
  return !currentPhase.isLeftPhase;
}

Rangea SimulatedRobot2D::getRotationRange(bool isLeftPhase, const Pose2f& walkSpeedRatio) const
{
  const float insideTurnRatio = 0.33f;

  const float innerTurn = 2.f * insideTurnRatio * maxSpeed.rotation * std::abs(walkSpeedRatio.rotation) * baseWalkPeriod;
  const float outerTurn = 2.f * (1.f - insideTurnRatio) * maxSpeed.rotation * std::abs(walkSpeedRatio.rotation) * baseWalkPeriod;
  return Rangea(isLeftPhase ? -innerTurn : -outerTurn, isLeftPhase ? outerTurn : innerTurn);
}

void SimulatedRobot2D::getTranslationRectangle(float rotation, const Pose2f& walkSpeedRatio, Vector2f& backRight, Vector2f& frontLeft) const
{
  const float reduceTranslationFromRotation = 0_deg;
  const float noTranslationFromRotation = 24_deg;
  const float sidewaysWalkPeriodIncreaseFactor = 0.f;

  backRight.x() = -1000.f;
  frontLeft.x() = 1000.f;
  backRight.y() = -1000.f;
  frontLeft.y() = 1000.f;

  // Limit to maximum speed (which is influenced by the rotation).
  const float tFactor = std::max(0.f, 1.f - std::max(0.f, (std::abs(rotation) - reduceTranslationFromRotation) / (noTranslationFromRotation - reduceTranslationFromRotation)));
  backRight.x() = std::max(backRight.x(), tFactor * -maxSpeedBackwards * baseWalkPeriod * std::abs(walkSpeedRatio.translation.x()));
  backRight.y() = std::max(backRight.y(), tFactor * -2.f * maxSpeed.translation.y() * (baseWalkPeriod + sidewaysWalkPeriodIncreaseFactor * maxSpeed.translation.y() * std::abs(walkSpeedRatio.translation.y())) * std::abs(walkSpeedRatio.translation.y()));
  frontLeft.x() = std::min(frontLeft.x(), tFactor * maxSpeed.translation.x() * baseWalkPeriod * std::abs(walkSpeedRatio.translation.x()));
  frontLeft.y() = std::min(frontLeft.y(), tFactor * 2.f * maxSpeed.translation.y() * (baseWalkPeriod + sidewaysWalkPeriodIncreaseFactor * maxSpeed.translation.y() * std::abs(walkSpeedRatio.translation.y())) * std::abs(walkSpeedRatio.translation.y()));

  // (0,0) must be part of the rectangle.
  backRight.x() = std::min(backRight.x(), -.01f);
  frontLeft.x() = std::max(frontLeft.x(), .01f);
  backRight.y() = std::min(backRight.y(), -.01f);
  frontLeft.y() = std::max(frontLeft.y(), .01f);
}
