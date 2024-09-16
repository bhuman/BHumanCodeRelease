/**
 * @file SimulatedNao/LocalConsole.h
 *
 * Declaration of LocalConsole.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "SimulatedNao/RobotConsole.h"
#include "Representations/Communication/GameControllerData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include <memory>

class Debug;

/**
 * @class LocalConsole
 *
 * A thread that is instantiated to either directly control a physical robot,
 * a simulated one, or to replay a log file.
 */
class LocalConsole : public RobotConsole
{
private:
  CameraImage cameraImage; /**< The simulated camera image sent to the robot code. */
  CameraInfo cameraInfo; /**< The information about the camera that took the image sent to the robot code. */
  JointSensorData jointSensorData; /**< The simulated joint measurements sent to the robot code. */
  FsrSensorData fsrSensorData; /**< The simulated inertia sensor data sent to the robot code. */
  RawInertialSensorData rawInertialSensorData; /**< The simulated inertia sensor data sent to the robot code. */
  MotionInfo motionInfo; /**< The simulated motion info sent to the robot code (2D only). */
  Pose2f robotPose; /**< The robot's pose, used for some internal computations. */
  GroundTruthWorldState worldState; /**< The current world state of the simulation scene, sent to the robot code. */
  GroundTruthOdometryData odometryData; /**< The simulated odometry data sent to the robot code. */
  GameControllerData gameControllerData; /**< The simulated game controller data sent to the robot code. */
  Whistle whistle; /**< The simulated whistle sent to the robot code. */
  unsigned nextImageTimestamp = 0; /**< The theoretical timestamp of the next image to be calculated. */
  unsigned imageLastTimestampSent = 0; /**< The timestamp of the last sent image. */
  unsigned jointLastTimestampSent = 0; /**< The timestamp of the last sent joint data. */
  bool imageCalculated = false; /**< Whether \c cameraImage contains a rendered image. */
  Semaphore updateSignal; /**< A signal used for synchronizing main() and update(). */
  Semaphore updatedSignal; /**< A signal used for yielding processing time to main(). */

public:
  /**
   * The constructor.
   * @param settings The settings for this robot.
   * @param robotName The name of this robot.
   * @param ctrl A pointer to the controller object.
   * @param logFile The log file name to replay or an empty string to create a simulated robot.
   * @param debug The debug connection of the robot with the simulator.
   */
  LocalConsole(const Settings& settings, const std::string& robotName, ConsoleRoboCupCtrl* ctrl, const std::string& logFile, Debug* debug);

  /**
   * The function is called once before the first main().
   * It increases the priority of this thread.
   */
  void init() override;

  /**
   * The function is called from the framework once in every frame.
   */
  bool main() override;

  /**
   * The function must be called to exchange data with SimRobot.
   * It sends the motor commands to SimRobot and acquires new sensor data.
   */
  void update() override;

private:
  /**
   * The function connects the robot to the returned receiver.
   *
   * @param debug The debug connection of the robot.
   * @return The receiver connected to the robot.
   */
  DebugReceiver<MessageQueue>* connectReceiverWithRobot(Debug* debug);

  /**
   * The function connects the robot to the returned sender.
   *
   * @param debug The debug connection of the robot.
   * @return The sender connected to the robot.
   */
  DebugSender<MessageQueue>* connectSenderWithRobot(Debug* debug) const;
};
