/**
 * @file Controller/LocalRobot.h
 *
 * Declaration of LocalRobot.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Controller/RobotConsole.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/MotionControl/OdometryData.h"
#include "SimulatedRobot.h"

class Debug;

/**
 * @class LocalRobot
 *
 * A thread that is instantiated to either directly control a physical robot,
 * a simulated one, or to replay a log file.
 */
class LocalRobot : public RobotConsole
{
private:
  CameraImage cameraImage; /**< The simulated camera image sent to the robot code. */
  CameraInfo cameraInfo; /**< The information about the camera that took the image sent to the robot code. */
  JointSensorData jointSensorData; /**< The simulated joint measurements sent to the robot code. */
  FsrSensorData fsrSensorData; /**< The simulated inertia sensor data sent to the robot code. */
  InertialSensorData inertialSensorData; /**< The simulated inertia sensor data sent to the robot code. */
  Pose2f robotPose; /**< The robot's pose, used for some internal computations. */
  GroundTruthWorldState worldState; /**< The current world state of the simulation scene, sent to the robot code. */
  GroundTruthOdometryData odometryData; /**< The simulated odometry data sent to the robot code. */
  unsigned nextImageTimestamp = 0; /**< The theoretical timestamp of the next image to be calculated. */
  unsigned imageLastTimestampSent = 0; /**< The timestamp of the last sent image. */
  unsigned jointLastTimestampSent = 0; /**< The timestamp of the last sent joint data. */
  SimulatedRobot simulatedRobot; /**< The interface to simulated objects. */
  Semaphore updateSignal; /**< A signal used for synchronizing main() and update(). */
  Semaphore updatedSignal; /**< A signal used for yielding processing time to main(). */
  SimRobotCore2::Body* puppet = nullptr; /**< A pointer to the puppet when there is one during log file replay. Otherwise 0. */

public:
  /**
   * The constructor.
   * @param debug The debug connection of the robot with the simulator.
   */
  LocalRobot(Debug* debug);

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
