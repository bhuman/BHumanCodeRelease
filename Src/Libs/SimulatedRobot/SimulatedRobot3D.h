/**
 * @file SimulatedRobot3D.h
 *
 * This file declares the interface to a robot with the 3D simulation core.
 *
 * @author Arne Hasselbring et al
 */

#pragma once

#include "SimulatedRobot/SimulatedRobot.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/MotorParameters.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "RobotParts/Joints.h"
#include <SimRobotCore3.h>
#include <random>

class SimulatedRobot3D : public SimulatedRobot
{
public:
  explicit SimulatedRobot3D(SimRobot::Object* robot);
  ~SimulatedRobot3D();

protected:
  void getRobotPose(Pose2f& robotPose) const override;
  void getTorsoMatrix(TorsoMatrix& torsoMatrix) override;
  void getImage(CameraImage& cameraImage, CameraInfo& cameraInfo) override;
  void getCameraInfo(CameraInfo& cameraInfo) override;
  void setJointCalibration(const JointCalibration& jointCalibration) override;
  void getAndSetJointData(const JointRequest& jointRequest, JointSensorData& jointSensorData, const MotorParameters& motorParameters) const override;
  void setJointRequest(const JointRequest& jointRequest, const MotorParameters& motorParameters, const bool isPuppet) const override;
  void getSensorData(InertialSensorData& inertialSensorData) override;
  void getAndSetMotionData(const MotionRequest& motionRequest, MotionInfo& motionInfo) override;
  void moveRobot(const Vector3f& pos, const Vector3f& rot, bool changeRotation, bool resetDynamics) override;
  void enablePhysics(bool enable) override;
  void enableGravity(bool enable) override;

  void enableSensorWhiteNoise(const bool enable) override;

  void enableSensorDiscretization(const bool enable) override;
  bool getPose2f(const SimRobot::Object* obj, Pose2f& pose) const override;
  void getPose3f(const SimRobot::Object* obj, Pose3f& pose) const override;

private:
  static SimRobotCore3::SensorPort* activeCameras[robotsPerTeam * 2]; /**< An array of all activated cameras */
  static unsigned activeCameraCount; /**< Total count of constructed cameras */
  unsigned activeCameraIndex; /**< Index of this robot in the \c activeCameras array */

  SimRobot::Object* jointSensors[Joints::numOfJoints] = {nullptr}; /**< The handles to the sensor ports of the joints. */
  SimRobot::Object* jointVelocitySensors[Joints::numOfJoints] = {nullptr}; /**< The handles to the velocity sensor ports of the joints. */
  SimRobot::Object* jointTorqueSensors[Joints::numOfJoints] = {nullptr}; /**< The handles to the torque sensor ports of the joints. */
  SimRobot::Object* jointActuators[Joints::numOfJoints] = {nullptr}; /**< The handles to the actuator ports of the joints. */
  SimRobot::Object* colorCameraSensor = nullptr; /**< The handle to the sensor port of the color camera. */
  SimRobot::Object* depthCameraSensor = nullptr; /**< The handle to the sensor port of the depth camera if one exists. */
  SimRobot::Object* accSensor = nullptr; /**< The handle to the sensor port of the virtual accelerometer. */
  SimRobot::Object* gyroSensor = nullptr; /**< The handle to the sensor port of the virtual gyroscope. */
  SimRobot::Object* leftFoot = nullptr; /**< The simulated left foot of the robot. */
  SimRobot::Object* rightFoot = nullptr; /**< The simulated right foot of the robot. */

  JointCalibration jointCalibration; /**< The simulated robot is perfectly calibrated, but this is useful for testing calibration. */
  CameraInfo cameraInfo; /**< Information about the camera. */
  RobotDimensions robotDimensions;

  std::random_device rand {};
  std::default_random_engine randomGenerator {rand()};
  std::normal_distribution<float> generalNormalDistribution {0.f, 1.f};

  const Angle gyroVariance = 0.00000387f; /**< Variance of the gyro (in (rad / s)²). */
  const float accVariance = 0.000374f; /**< White noise variance of the accelerometer measurements (in (m/s²)²). */

  const Angle jointDiscretizationStep = 360_deg / (1 << 14);

  bool useWhiteNoise = true; /**< If true white noise is applied to the sensor readings. */
  bool useDiscretization = true; /**< If true the discretization of the sensor data is simulated. */

  /**
   * Discretizes the value if this type of distortion is active.
   * @param value value to discretize
   * @param discretizationStep step size of the discretization
   * @return the discretized value
   */
  float applyDiscretization(float value, float discretizationStep) const;

  /**
   * Applies Gaussian noise to the given value if this type of distortion is active.
   * @param value base value to add Gaussian noise to
   * @param variance variance of the added distortion
   * @return value + noise
   */
  float applyWhiteNoise(float value, float variance);
};
