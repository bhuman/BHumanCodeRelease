/**
 * @file SimulatedRobot3D.h
 *
 * This file declares the interface to a robot with the 3D simulation core.
 *
 * @author Arne Hasselbring et al
 */

#pragma once

#include "SimulatedNao/SimulatedRobot.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Streaming/EnumIndexedArray.h"
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
  void getAndSetJointData(const JointRequest& jointRequest, JointSensorData& jointSensorData) const override;
  void setJointRequest(const JointRequest& jointRequest, const bool isPuppet) const override;
  void toggleCamera() override;
  void getSensorData(FsrSensorData& fsrSensorData, RawInertialSensorData& rawInertialSensorData) override;
  void getAndSetMotionData(const MotionRequest& motionRequest, MotionInfo& motionInfo) override;
  void moveRobot(const Vector3f& pos, const Vector3f& rot, bool changeRotation, bool resetDynamics) override;
  void enablePhysics(bool enable) override;
  void enableGravity(bool enable) override;

  void enableSensorWhiteNoise(const bool enable) override;

  void enableSensorDelay(const bool enable) override;

  void enableSensorDiscretization(const bool enable) override;
  bool getPose2f(const SimRobot::Object* obj, Pose2f& pose) const override;
  void getPose3f(const SimRobot::Object* obj, Pose3f& pose) const override;

private:
  static SimRobotCore3::SensorPort* activeCameras[robotsPerTeam * 2]; /**< An array of all activated cameras */
  static unsigned activeCameraCount; /**< Total count of constructed cameras */
  unsigned activeCameraIndex; /**< Index of this robot in the \c activeCameras array */

  SimRobot::Object* jointSensors[Joints::numOfJoints] = {nullptr}; /**< The handles to the sensor ports of the joints. */
  SimRobot::Object* jointVelocitySensors[Joints::numOfJoints] = {nullptr}; /**< The handles to the velocity sensor ports of the joints. */
  SimRobot::Object* jointActuators[Joints::numOfJoints] = {nullptr}; /**< The handles to the actuator ports of the joints. */
  SimRobot::Object* cameraSensor = nullptr; /**< The handle to the sensor port of the selected camera. */
  SimRobot::Object* upperCameraSensor = nullptr; /**< The handle to the sensor port of the upper camera. */
  SimRobot::Object* lowerCameraSensor = nullptr; /**< The handle to the sensor port of the lower camera. */
  SimRobot::Object* accSensor = nullptr; /**< The handle to the sensor port of the virtual accelerometer. */
  SimRobot::Object* gyroSensor = nullptr; /**< The handle to the sensor port of the virtual gyroscope. */
  SimRobot::Object* leftFoot = nullptr; /**< The simulated left foot of the robot. */
  SimRobot::Object* rightFoot = nullptr; /**< The simulated right foot of the robot. */

  JointCalibration jointCalibration; /**< The simulated robot is perfectly calibrated, but this is useful for testing calibration. */
  ENUM_INDEXED_ARRAY(CameraInfo, CameraInfo::Camera) cameraInfos; /**< Information about the upper camera. */
  RobotDimensions robotDimensions;

  std::random_device rand {};
  std::default_random_engine randomGenerator {rand()};
  std::normal_distribution<float> generalNormalDistribution {0.f, 1.f};

  bool newGyroMeasurement = false;

  const Angle gyroVariance = 0.00000387f; /**< Variance of the gyro (in (rad / s)²). */
  const float accVariance = 0.000374f; /**< White noise variance of the accelerometer measurements (in (m/s²)²). */

  const Angle jointDiscretizationStep = 360_deg / (1 << 12);

  bool useWhiteNoise = true; /**< If true white noise is applied to the sensor readings. */
  bool useTimeDelay = true; /**< If true a time delay of the sensor data is simulated. This includes the simulation of alternating new data from the IMU. */
  bool useDiscretization = true; /**< If true the discretization of the sensor data is simulated. */

  RawInertialSensorData lastInertialData; /**< Inertial data from the last frame that was skipped */

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
