/**
 * @file SimulatedRobot2D.h
 *
 * This file declares the interface to a robot with the 2D simulation core.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Controller/SimulatedRobot.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include <SimRobotCore2D.h>

class SimulatedRobot2D : public SimulatedRobot, public SimRobotCore2D::CollisionCallback
{
public:
  explicit SimulatedRobot2D(SimRobot::Object* robot);

protected:
  void getRobotPose(Pose2f& robotPose) const override;
  void getImage(CameraImage& cameraImage, CameraInfo& cameraInfo) override;
  void getCameraInfo(CameraInfo& cameraInfo) override;
  void setJointCalibration(const JointCalibration& jointCalibration) override;
  void getAndSetJointData(const JointRequest& jointRequest, JointSensorData& jointSensorData) const override;
  void setJointRequest(const JointRequest& jointRequest) const override;
  void toggleCamera() override;
  void getSensorData(FsrSensorData& fsrSensorData, InertialSensorData& inertialSensorData) override;
  void getAndSetMotionData(const MotionRequest& motionRequest, MotionInfo& motionInfo) override;
  void moveRobot(const Vector3f& pos, const Vector3f& rot, bool changeRotation) override;
  void enablePhysics(bool enable) override;
  bool getPose2f(const SimRobot::Object* obj, Pose2f& pose) const override;
  void getPose3f(const SimRobot::Object* obj, Pose3f& pose) const override;

  void collided(SimRobotCore2D::Geometry& geom1, SimRobotCore2D::Geometry& geom2) override;

private:
  void requestPlayDead();
  void requestStand(bool high);
  void requestWalkAtAbsoluteSpeed(const Pose2f& walkSpeed);
  void requestWalkAtRelativeSpeed(const Pose2f& walkSpeed);
  void requestWalkToPose(const Pose2f& walkTarget, const Pose2f& walkSpeed, bool keepTargetRotation, const MotionRequest::ObstacleAvoidance& obstacleAvoidance);
  void requestWalkToBallAndKick(const BallState& ballEstimate, Angle targetDirection, KickInfo::KickType kickType, float kickPower, bool alignPrecisely, const Pose2f& walkSpeed, const MotionRequest::ObstacleAvoidance& obstacleAvoidance);
  void requestWalkToBall(const Pose2f& kickPose, const Vector2f& ballPosition, const Pose2f& walkSpeed, const MotionRequest::ObstacleAvoidance& obstacleAvoidance);
  void requestDribble(const BallState& ballEstimate, Angle targetDirection, const Pose2f& walkSpeed, const MotionRequest::ObstacleAvoidance& obstacleAvoidance);
  void requestGetUp();
  void requestKeyframeMotion(const KeyframeMotionRequest& keyframeMotionRequest);

  void executePlayDead(MotionInfo& motionInfo);
  void executeStand(MotionInfo& motionInfo);
  void executeWalk(MotionInfo& motionInfo);
  void executeKick(MotionInfo& motionInfo);
  void executeFall(MotionInfo& motionInfo);
  void executeGetUp(MotionInfo& motionInfo);
  void executeKeyframeMotion(MotionInfo& motionInfo);

  bool isNextLeftPhase(bool shouldBeLeft) const;
  Rangea getRotationRange(bool isLeftPhase, const Pose2f& walkSpeedRatio) const;
  void getTranslationRectangle(float rotation, const Pose2f& walkSpeedRatio, Vector2f& backRight, Vector2f& frontLeft) const;

  struct KickDescriptor
  {
    KickInfo::KickType type;
    float impactT; /**< The time within the phase when the ball gets the momentum. */
    float duration; /**< The duration for actual kick phases. */
    std::vector<Pose2f> steps; /**< The steps for in-walk-kicks (currently only one step is supported). */
    Rangef distanceRange; /**< The range of distances that can be reached by this kick. */
    float distanceDeviation; /**< The deviation of the distance as ratio of the actual distance. */
    float direction; /**< The direction in which the ball travels. */
    float directionDeviation; /**< The deviation of the direction. */
  };

  ENUM_INDEXED_ARRAY(CameraInfo, CameraInfo::Camera) cameraInfos; /**< Information about the upper camera. */
  CameraInfo::Camera currentCamera = CameraInfo::upper; /**< The camera from which the next camera info will be returned. */
  BallSpecification ballSpecification; /**< The ball specification. */
  KickInfo kickInfo; /**< The kick info. */
  std::array<KickDescriptor, KickInfo::numOfKickTypes> kicks; /**< How to model kicks. */
  SimRobotCore2D::Geometry* robotGeometry = nullptr; /**< The geometry (i.e. the collision body) of this robot. */

  struct PhaseInfo
  {
    MotionPhase::Type motion = MotionPhase::playDead; /**< The motion phase type. */
    float t = 0.f; /**< The current time within the phase. */
    float duration = 0.f; /**< The duration of the phase. */
    Pose2f step; /**< For walk phases: The step to make in this phase. */
    bool isLeftPhase = false; /**< For walk phases: Is the left foot the swing foot? */
    const KickDescriptor* kick = nullptr; /**< For walk-kick or kick phases: The kick descriptor in this phase. */
    float kickPower = 1.f; /**< For walk-kick or kick phases: The kick power in this phase. */
    bool performedAction = false; /**< For phases that do some action at a specific time point: Whether that action has been performed. */
    KeyframeMotionRequest keyframeMotionRequest; /**< For keyframe motion phases: The keyframe motion. */
  };

  PhaseInfo currentPhase; /**< The current phase. */

  std::unordered_set<SimRobotCore2D::Body*> collisionsThisFrame; /**< The bodies with which this robot collided during this frame. */
  std::unordered_map<SimRobotCore2D::Body*, RingBufferWithSum<float, 60>> collisionBuffer; /**< Ring buffers of collisions during the previous frames for all other bodies. */

  const float baseWalkPeriod = 0.25f;
  const Pose2f maxSpeed = Pose2f(70_deg, 250.f, 180.f); /**< The maximum speed of the robot. */
  static constexpr float maxSpeedBackwards = 200.f; /**< The maximum backwards speed of the robot. */
  static constexpr float zero[2] = {0.f, 0.f}; /**< A zero vector that can be passed to \c SimRobotCore2D::Body::setVelocity. */
};
