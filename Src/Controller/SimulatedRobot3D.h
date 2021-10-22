/**
 * @file SimulatedRobot3D.h
 *
 * This file declares the interface to a robot with the 3D simulation core.
 *
 * @author Arne Hasselbring et al
 */

#pragma once

#include "Controller/SimulatedRobot.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/RobotParts/Joints.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include <SimRobotCore2.h>

class SimulatedRobot3D : public SimulatedRobot
{
public:
  explicit SimulatedRobot3D(SimRobot::Object* robot);
  ~SimulatedRobot3D();

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

private:
  static SimRobotCore2::SensorPort* activeCameras[12]; /**< An array of all activated cameras */
  static unsigned activeCameraCount; /**< Total count of constructed cameras */
  unsigned activeCameraIndex; /**< Index of this robot in the \c activeCameras array */

  SimRobot::Object* jointSensors[Joints::numOfJoints] = {nullptr}; /**< The handles to the sensor ports of the joints. */
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
};
