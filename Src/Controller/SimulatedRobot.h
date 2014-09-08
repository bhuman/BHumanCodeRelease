/**
* @file Controller/SimulatedRobot.h
* Declaration of class SimulatedRobot for SimRobotQt.
* @author Colin Graf
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include <SimRobotCore2.h>
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/USRequest.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/Infrastructure/CameraResolution.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/Pose3D.h"

class RoboCupCtrl;
class Pose2D;
class OdometryData;
class Image;
class SensorData;
class JointData;
class OrientationData;
class ImageInfo;
class ImageRequest;
class GroundTruthWorldState;

/**
* An interface to a simulated robot (and its ball).
*/
class SimulatedRobot
{
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
  void getRobotPose(Pose2D& robotPose) const;

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
  void getOdometryData(const Pose2D& robotPose, OdometryData& odometryData) const;

  /**
  * Determines the ball position in the scene (not considering the robot's color).
  * @param ballPosition The position of the ball
  */
  static void getAbsoluteBallPosition(Vector2<>& ballPosition);

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
  * Determines the current joint data of the simulated robot and sets new ones.
  * @param jointRequest The joint data to set.
  * @param jointData The determined joint data.
  */
  void getAndSetJointData(const JointData& jointRequest, JointData& jointData) const;

  /**
  * Sets the values off all joint actuators.
  * @param jointRequest The joint data to set.
  */
  void setJointData(const JointData& jointRequest) const;

  /**
   * Toggles between the two cameras.
   */
  void toggleCamera();

  /**
  * Determines the sensor data of the simulated robot.
  * @param sensorData The determined sensor data.
  * @param usRequest The request that determines which sonars are read.
  */
  void getSensorData(SensorData& sensorData, const USRequest& usRequest);

  /**
  * Determines "ground truth" orientation data.
  * @param sensorData The current set of sensor data.
  * @param orientationData The determined orientation data.
  */
  void getOrientationData(const SensorData& sensorData, OrientationData& orientationData);

  /**
  * Moves and rotates the robot to an absolute pose
  * @param pos The position to move the robot to
  * @param rot The target rotation (as angle axis; in radian)
  * @param changeRotation Whether the rotation of the robot should be changed or not
  */
  void moveRobot(const Vector3<>& pos, const Vector3<>& rot, bool changeRotation);

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
  static void moveBall(const Vector3<>& pos, bool resetDynamics = false);

  /**
  * Determines the two-dimensional position of a SimRobot object without team color rotation.
  * @param obj The object of which the position will be determined.
  */
  static Vector2<> getPosition(SimRobot::Object* obj);

  /**
  * Determines whether a robot is member of the blue or red team
  * @param obj The robot
  * @return \c true if the robot is in the blue team; \c false otherwise
  */
  static bool isBlue(SimRobot::Object* obj);

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

  Pose3D lastRobotPose3D, /**< The last robot pose in 3-D. */
         robotPose3D; /**< A buffer for the current 3-D robot pose. */
  unsigned int lastTimeStamp; /** The time stamp of the previous iteration. */

  SimRobot::Object* jointSensors[JointData::numOfJoints]; /**< The handles to the sensor ports of the joints. */
  SimRobot::Object* jointActuators[JointData::numOfJoints]; /**< The handles to the actuator ports of the joints. */
  SimRobot::Object* cameraSensor; /**< The handle to the sensor port of the selected camera. */
  SimRobot::Object* upperCameraSensor; /**< The handle to the sensor port of the upper camera. */
  SimRobot::Object* lowerCameraSensor; /**< The handle to the sensor port of the lower camera. */
  SimRobot::Object* accSensor;    /**< The handle to the sensor port of the virtual accelerometer. */
  SimRobot::Object* gyroSensor;   /**< The handle to the sensor port of the virtual gyrosope. */
  SimRobot::Object* leftUsSensor;     /** The handle to the sensor port of the virtual us sensor */
  SimRobot::Object* rightUsSensor;    /** The handle to the sensor port of the virtual us sensor */
  SimRobot::Object* centerLeftUsSensor; /** The handle to the sensor port of the virtual us sensor */
  SimRobot::Object* centerRightUsSensor; /** The handle to the sensor port of the virtual us sensor */

  static SimRobotCore2::SensorPort* activeCameras[14]; /**< An array of all activated cameras */
  static unsigned activeCameraCount; /**< Total count of constructed cameras */
  unsigned activeCameraIndex; /**< Index of this robot in the \c activeCameras array */

  JointCalibration jointCalibration; /**< The simulated robot is perfectly calibrated, but we need the signs. */
  CameraInfo upperCameraInfo; /**< Information about the upper camera. */
  CameraInfo lowerCameraInfo; /**< Information about the lower camera. */
  CameraIntrinsics cameraIntrinsics;
  CameraResolution cameraResolution;

  /**
   * Adds jitter to a sensor value.
   */
  float addJitter(float value);

  /**
  * Determines the two-dimensional pose of a SimRobot object without team color rotation.
  * @param obj The object of which the pose will be determined.
  * @param pose2D The two-dimensional pose of the specified object.
  */
  void getPose2D(SimRobot::Object* obj, Pose2D& pose2D) const;

  /**
  * Determines the three-dimensional pose of a SimRobot object without team color rotation.
  * @param obj The object of which the pose will be determined.
  * @param pose3D The three-dimensional pose of the specified object.
  */
  void getPose3D(SimRobot::Object* obj, Pose3D& pose3D) const;

  /**
  * Determines a robot's number
  * @param obj The robot
  * @return The number
  */
  static int getNumber(SimRobot::Object* obj);
};
