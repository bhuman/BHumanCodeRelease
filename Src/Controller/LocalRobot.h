/**
 * @file Controller/LocalRobot.h
 *
 * Declaration of LocalRobot.
 *
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Controller/RobotConsole.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/MotionControl/OdometryData.h"
#include "SimulatedRobot.h"

/**
 * @class LocalRobot
 *
 * A process that is instantiated to either directly control a physical robot,
 * a simulated one, or to replay a logfile.
 */
class LocalRobot : public RobotConsole
{
  Receiver<MessageQueue> theDebugReceiver;
  DebugSender<MessageQueue> theDebugSender;
  Image image; /**< The simulated image sent to the robot code. */
  CameraInfo cameraInfo; /**< The information about the camera that took the image sent to the robot code. */
  JointSensorData jointSensorData; /**< The simulated joint measurements sent to the robot code. */
  FsrSensorData fsrSensorData; /**< The simulated inertia sensor data sent to the robot code. */
  InertialSensorData inertialSensorData; /**< The simulated inertia sensor data sent to the robot code. */
  Pose2f robotPose; /**< The robot's pose, used for some internal computations. */
  GroundTruthWorldState worldState; /**< The current world state of the simulation scene, sent to the robot code. */
  GroundTruthOdometryData odometryData; /**< The simulated odometry data sent to the robot code. */
  unsigned nextImageTimeStamp = 0; /**< The theoretical timestamp of the next image to be calculated. */
  unsigned imageLastTimeStampSent = 0; /**< The timestamp of the last sent image. */
  unsigned jointLastTimeStampSent = 0; /**< The timestamp of the last sent joint data. */
  SimulatedRobot simulatedRobot; /**< The interface to simulated objects. */
  Semaphore updateSignal; /**< A signal used for synchronizing main() and update(). */
  Semaphore updatedSignal; /**< A signal used for yielding processing time to main(). */
  SimRobotCore2::Body* puppet = nullptr; /**< A pointer to the puppet when there is one during logfile replay. Otherwise 0. */

public:
  LocalRobot();

  /**
   * The function is called from the framework once in every frame
   */
  bool main() override;

  /**
   * The function must be called to exchange data with SimRobot.
   * It sends the motor commands to SimRobot and acquires new sensor data.
   */
  void update() override;
};
