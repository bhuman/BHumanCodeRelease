/**
 * @file SimulatedNao/SimulatedRobot.h
 * Declaration of class SimulatedRobot for SimRobotQt.
 * @author Colin Graf
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Math/Eigen.h"
#include <SimRobot.h>
#include <vector>

struct CameraImage;
struct CameraInfo;
struct FsrSensorData;
struct GroundTruthWorldState;
struct JointCalibration;
struct JointRequest;
struct JointSensorData;
struct MotionInfo;
struct MotionRequest;
struct OdometryData;
struct Pose2f;
struct Pose3f;
struct RawInertialSensorData;
struct TorsoMatrix;

/**
 * An interface to a simulated robot (and its ball).
 */
class SimulatedRobot
{
private:
  int robotNumber = -1; /**< The number of this robot */
  std::vector<SimRobot::Object*> firstTeamRobots; /**< The simulated robots in the first team (excluding this robot). */
  std::vector<SimRobot::Object*> secondTeamRobots; /**< The simulated robots in the second team (excluding this robot). */
  mutable Vector3f lastBallPosition; /**< The ball position at the time when \c getWorldState was called last. */
  mutable unsigned lastBallTime = 0; /**< The simulated time when \c getWorldState was called last. */
  mutable bool hadVelocity = false;
  mutable Angle curveVel = 0_deg;

protected:
  static SimRobot::Object* ball; /**< The simulated ball. */
  SimRobot::Object* robot = nullptr; /**< The simulated robot object. */
  bool firstTeam = false; /**< Whether this robot is in the first team or not. */

public:
  /**
   * Initializes interface for the given robot.
   * @param robot The robot to initialize the interface for.
   */
  explicit SimulatedRobot(SimRobot::Object* robot);

  /** Virtual destructor for polymorphism. */
  virtual ~SimulatedRobot() = default;

  /**
   * Sets the only ball used to create the ball model.
   */
  static void setBall(SimRobot::Object* ball);

  /**
   * Determines the pose of the simulated robot.
   * @param robotPose The determined pose of the robot.
   */
  virtual void getRobotPose(Pose2f& robotPose) const = 0;

  /**
   * Determines the torso matrix of the simulated robot.
   * @param torsoMatrix The determined torso matrix of the robot.
   */
  virtual void getTorsoMatrix(TorsoMatrix& torsoMatrix) = 0;

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
   * Determines the camera image of the simulated robot.
   * @param cameraImage The determined image.
   * @param cameraInfo The information about the camera that took the image.
   */
  virtual void getImage(CameraImage& cameraImage, CameraInfo& cameraInfo) = 0;

  /**
   * Determines the camera information (in case no images are generated) of the simulated robot.
   * @param cameraInfo The information about the camera that took the image (in theory).
   */
  virtual void getCameraInfo(CameraInfo& cameraInfo) = 0;

  /**
   * Sets the values of the JointCalibration.
   * @param jointCalibration The joint calibration to set.
   */
  virtual void setJointCalibration(const JointCalibration& jointCalibration) = 0;

  /**
   * Determines the current joint angles of the simulated robot and sets new ones.
   * @param jointRequest The joint request to set.
   * @param jointAngles The determined joint angles.
   */
  virtual void getAndSetJointData(const JointRequest& jointRequest, JointSensorData& jointSensorData) const = 0;

  /**
   * Sets the values of all joint actuators.
   * @param jointRequest The joint request to set.
   */
  virtual void setJointRequest(const JointRequest& jointRequest, const bool isPuppet) const = 0;

  /**
   * Toggles between the two cameras.
   */
  virtual void toggleCamera() = 0;

  /**
   * Determines the sensor data of the simulated robot.
   * @param fsrSensorData The determined FSR sensor data.
   * @param rawInertialSensorData The determined inertial sensor data.
   */
  virtual void getSensorData(FsrSensorData& fsrSensorData, RawInertialSensorData& inertialSensorData) = 0;

  /**
   * Does the abstract motion emulation.
   * @param motionRequest The requested motion.
   * @param motionInfo The actually executed motion.
   */
  virtual void getAndSetMotionData(const MotionRequest& motionRequest, MotionInfo& motionInfo) = 0;

  /**
   * Moves and rotates the robot to an absolute pose
   * @param pos The position to move the robot to
   * @param rot The target rotation (as euler angles; in radian)
   * @param changeRotation Whether the rotation of the robot should be changed or not
   * @param resetDynamics Reset dynamics of object after moving.
   */
  virtual void moveRobot(const Vector3f& pos, const Vector3f& rot, bool changeRotation, bool resetDynamics = false) = 0;

  /**
   * Enables or disables the physics simulation of the body
   * @param enable Whether to enable or disable the physics simulation
   */
  virtual void enablePhysics(bool enable) = 0;

  /**
   * Enables or disables the gravity for the body
   * @param enable Whether to enable or disable gravity
   */
  virtual void enableGravity(bool enable) = 0;

  /**
   * Enables or disables the simulation white noise
   * @param enable Whether to enable or disable white noise
   */
  virtual void enableSensorWhiteNoise(const bool enable) = 0;

  /**
   * Enables or disables the simulation sensor delay
   * @param enable Whether to enable or disable sensor delay
   */
  virtual void enableSensorDelay(const bool enable) = 0;

  /**
   * Enables or disables the simulation discretization af sensor values
   * @param enable Whether to enable or disable discretization
   */
  virtual void enableSensorDiscretization(const bool enable) = 0;

  /**
   * Moves and rotates the robot to an pose within the coordinate system of its team
   * @param pos The position to move the robot to
   * @param rot The target rotation (as euler angles; in radian)
   * @param changeRotation Whether the rotation of the robot should be changed or not
   * @param resetDynamics Reset dynamics of object after moving.
   */
  void moveRobotPerTeam(const Vector3f& pos, const Vector3f& rot, bool changeRotation, bool resetDynamics = false);

  /**
   * Moves the ball to the given position within the coordinate system of the robot's team
   * @param pos The position to move the ball to
   * @param resetDynamics Reset dynamics of object after moving.
   */
  void moveBallPerTeam(const Vector3f& pos, bool resetDynamics = false);

  /**
   * Determines the ball position in the scene (not considering the robot's color).
   * @param ballPosition The position of the ball.
   * @return Whether there is a ball in the scene.
   */
  static bool getAbsoluteBallPosition(Vector2f& ballPosition);

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
  static Vector2f getPosition(const SimRobot::Object* obj);

  /**
   * Determines the two-dimensional position of this robot without team color rotation.
   */
  Vector2f getPosition() const {return getPosition(robot);}

  /**
   * Determines the three-dimensional position of a SimRobot object without team color rotation.
   * @param obj The object of which the position will be determined.
   */
  static Vector3f getPosition3D(const SimRobot::Object* obj);

  /**
   * Applies rolling friction to the ball (if the simulation core does not do it).
   * @param friction The negative acceleration of the ball (m/s^2).
   */
  static void applyBallFriction(float friction);

  /**
   * Determines whether a robot is member of the first or second team
   * @param obj The robot
   * @return \c true if the robot is in the first team; \c false otherwise
   */
  static bool isFirstTeam(const SimRobot::Object* obj);

  /**
   * Determines a robot's number
   * @param obj The robot
   * @return The number
   */
  static int getNumber(const SimRobot::Object* obj);

  static constexpr int robotsPerTeam = 20; /**< Actually the offset of the second team's robot object numbers in the simulation scene. */

protected:
  /**
   * Determines the two-dimensional pose of a SimRobot object without team color rotation.
   * @param obj The object of which the pose will be determined.
   * @param pose The two-dimensional pose of the specified object.
   * @return Is the robot upright?
   */
  virtual bool getPose2f(const SimRobot::Object* obj, Pose2f& pose) const = 0;

  /**
   * Determines the three-dimensional pose of a SimRobot object without team color rotation.
   * @param obj The object of which the pose will be determined.
   * @param pose The three-dimensional pose of the specified object.
   */
  virtual void getPose3f(const SimRobot::Object* obj, Pose3f& pose) const = 0;
};
