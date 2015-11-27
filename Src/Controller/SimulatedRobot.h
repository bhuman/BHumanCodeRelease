/**
 * @file Controller/SimulatedRobot.h
 * Declaration of class SimulatedRobot for SimRobotQt.
 * @author Colin Graf
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include <SimRobotCore2.h>
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/UsConfiguration.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/Infrastructure/CameraResolution.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Tools/Joints.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"

struct GroundTruthWorldState;
struct Image;
struct InertialSensorData;
struct JointRequest;
struct OdometryData;
struct USRequest;
struct UsSensorData;
struct Pose2f;

/**
 * An interface to a simulated robot (and its ball).
 */
class SimulatedRobot
{
private:
  SimRobot::Application* application;

  bool blue; /**< Whether this robot has blue as team color or not. */
  int robotNumber; /**< The number of this robot */
  SimRobot::Object* robot; /**< The simulated robot object. */
  SimRobot::Object* leftFoot; /**< The simulated left foot of the robot. */
  SimRobot::Object* rightFoot; /**< The simulated right foot of the robot. */
  static SimRobot::Object* ball; /**< The simulated ball. */
  std::vector<SimRobot::Object*> blueRobots; /**< The simulated blue robots (excluding this robot). */
  std::vector<SimRobot::Object*> redRobots; /**< The simulated red robots (excluding this robot). */

  SimRobot::Object* jointSensors[Joints::numOfJoints]; /**< The handles to the sensor ports of the joints. */
  SimRobot::Object* jointActuators[Joints::numOfJoints]; /**< The handles to the actuator ports of the joints. */
  SimRobot::Object* cameraSensor; /**< The handle to the sensor port of the selected camera. */
  SimRobot::Object* upperCameraSensor; /**< The handle to the sensor port of the upper camera. */
  SimRobot::Object* lowerCameraSensor; /**< The handle to the sensor port of the lower camera. */
  SimRobot::Object* accSensor;    /**< The handle to the sensor port of the virtual accelerometer. */
  SimRobot::Object* gyroSensor;   /**< The handle to the sensor port of the virtual gyrosope. */
  SimRobot::Object* leftUsSensor;     /** The handle to the sensor port of the virtual us sensor */
  SimRobot::Object* rightUsSensor;    /** The handle to the sensor port of the virtual us sensor */
  SimRobot::Object* centerLeftUsSensor; /** The handle to the sensor port of the virtual us sensor */
  SimRobot::Object* centerRightUsSensor; /** The handle to the sensor port of the virtual us sensor */

  static SimRobotCore2::SensorPort* activeCameras[12]; /**< An array of all activated cameras */
  static unsigned activeCameraCount; /**< Total count of constructed cameras */
  unsigned activeCameraIndex; /**< Index of this robot in the \c activeCameras array */

  JointCalibration jointCalibration; /**< The simulated robot is perfectly calibrated, but we need the signs. */
  CameraInfo upperCameraInfo; /**< Information about the upper camera. */
  CameraInfo lowerCameraInfo; /**< Information about the lower camera. */
  CameraIntrinsics cameraIntrinsics;
  CameraResolution cameraResolution;
  UsConfiguration usConfig;

public:
  /** Default constructor */
  SimulatedRobot();

  /** Destructor */
  ~SimulatedRobot();

  /**
   * Initializes iterface for the given robot.
   * @param robot The robot to initialize the interface for
   */
  void init(SimRobot::Object* robot);

  /**
   * Sets the only ball used to create the ball model.
   */
  static void setBall(SimRobot::Object* ball);

  /**
   * Determines the pose of the simulated robot.
   * @param robotPose The determined pose of the robot.
   */
  void getRobotPose(Pose2f& robotPose) const;

  /**
   * Determines all robot states as well as the ball state.
   * @param worldState The determined world state.
   */
  void getWorldState(GroundTruthWorldState& worldState) const;

  /**
   * Determines the odometry data of the simulated robot.
   * @param robotPose The pose of the robot.
   * @param odometryData The determined odometry data of the robot.
   */
  void getOdometryData(const Pose2f& robotPose, OdometryData& odometryData) const;

  /**
   * Determines the ball position in the scene (not considering the robot's color).
   * @param ballPosition The position of the ball
   */
  static void getAbsoluteBallPosition(Vector2f& ballPosition);

  /**
   * Determines the camera image of the simulated robot.
   * @param image The determined image.
   * @param cameraInfo The information about the camera that took the image.
   */
  void getImage(Image& image, CameraInfo& cameraInfo);

  /**
   * Determines the camera information (in case no images are generated) of the simulated robot.
   * @param cameraInfo The information about the camera that took the image (in theory).
   */
  void getCameraInfo(CameraInfo& cameraInfo);

  /**
   * Determines the current joint angles of the simulated robot and sets new ones.
   * @param jointRequest The joint request to set.
   * @param jointAngles The determined joint angles.
   */
  void getAndSetJointData(const JointRequest& jointRequest, JointSensorData& jointSensorData) const;

  /**
   * Sets the values off all joint actuators.
   * @param jointRequest The joint request to set.
   */
  void setJointRequest(const JointRequest& jointRequest) const;

  /**
   * Toggles between the two cameras.
   */
  void toggleCamera();

  /**
   * Determines the sensor data of the simulated robot.
   * @param inertialSensorData The determined inertial sensor data.
   * @param usSensorData The determined usSensorData sensor data.
   * @param usRequest The request that determines which sonars are read.
   */
  void getSensorData(InertialSensorData& inertialSensorData, UsSensorData& usSensorData, const USRequest& usRequest);

  /**
   * Moves and rotates the robot to an absolute pose
   * @param pos The position to move the robot to
   * @param rot The target rotation (as euler angles; in radian)
   * @param changeRotation Whether the rotation of the robot should be changed or not
   */
  void moveRobot(const Vector3f& pos, const Vector3f& rot, bool changeRotation);

  /**
   * Enables or disables the physics simulation of the body
   * @param enable Whether to enable or disable the physics simulation
   */
  void enablePhysics(bool enable);

  /**
   * Moves the ball to the given position
   * @param pos The position to move the ball to
   * @param resetDynamics Reset dynamics of object after moving.
   */
  static void moveBall(const Vector3f& pos, bool resetDynamics = false);

  /**
   * Determines the two-dimensional position of a SimRobot object without team color rotation.
   * @param obj The object of which the position will be determined.
   */
  static Vector2f getPosition(SimRobot::Object* obj);

  /**
   * Determines whether a robot is member of the blue or red team
   * @param obj The robot
   * @return \c true if the robot is in the blue team; \c false otherwise
   */
  static bool isBlue(SimRobot::Object* obj);

private:
  /**
   * Adds jitter to a sensor value.
   */
  float addUsJitter(float value);

  /**
   * Determines the two-dimensional pose of a SimRobot object without team color rotation.
   * @param obj The object of which the pose will be determined.
   * @param Pose2f The two-dimensional pose of the specified object.
   * @return Is the robot upright?
   */
  bool getPose2f(SimRobot::Object* obj, Pose2f& Pose2f) const;

  /**
   * Determines the three-dimensional pose of a SimRobot object without team color rotation.
   * @param obj The object of which the pose will be determined.
   * @param Pose3f The three-dimensional pose of the specified object.
   */
  void getPose3f(SimRobot::Object* obj, Pose3f& Pose3f) const;

  /**
   * Determines a robot's number
   * @param obj The robot
   * @return The number
   */
  static int getNumber(SimRobot::Object* obj);
};
