/**
 * @file BoosterProvider.h
 *
 * This file declares a module that connects to the Booster robot.
 * In playDead, the damping mode is commanded. In any other case,
 * the custom mode is used. When waiting for a connection to the
 * robot, the custom mode is requested as well, because the robot
 * seems to accept the damping mode when it is not completely ready.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/ModifiedJointRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/MotionControl/MotionRequest.h"

#if defined TARGET_ROBOT && (defined __arm64__ || defined __aarch64__)
#define TARGET_BOOSTER
#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/RobotStatusDdsMsg.h>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#endif

MODULE(BoosterProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(JointCalibration),
  USES(JointRequest),
  USES(JointSensorData),
  USES(ModifiedJointRequest),
  REQUIRES(MotionRequest),
  PROVIDES(FrameInfo),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(RawInertialSensorData),
  PROVIDES(SystemSensorData),
  LOADS_PARAMETERS(
  {
    STREAMABLE(ControlParameter,
    {,
      (float) p, /**< p factor for a joint controller. */
      (float) d, /**< d factor for a joint controller. */
      (float) maxTorque,
    }),

    (unsigned char) minHot, /**< Minimum temperature for status hot (in °C). */
    (unsigned char) minVeryHot, /**< Minimum temperature for status very hot (in °C). */
    (unsigned char) minCriticallyHot, /**< Minimum temperature for status critically hot (in °C). */
    (int) maxDelayForFrameData, /**< The maximum time waited for new frame data. */
    (std::vector<Joints::Joint>) jointMapping, /**< The mapping from Booster joints indices to B-Human joint indices. */
    (std::vector<ControlParameter>) controlParameters, /**< The controller factors for the different joints. */
    (bool) setTorque,
    (Rangea) shoulderPitchRangeAtStart, /**< At start the shoulder pitches should be insides this range. */
    (int) armsWarningTime, /**< Warn about the arm positions after this much time passed. */
  }),
});

class BoosterProvider : public BoosterProviderBase
{
  DECLARE_SYNC; /**< Data is received in a different thread, so synchronize access. */
#ifndef TARGET_BOOSTER
  thread_local
#endif
  static BoosterProvider* theInstance; /**< The only instance of this module. */
  JointSensorData jointSensorData; /**< The joint sensor data received. */
  RawInertialSensorData rawInertialSensorData; /**< The inertial sensor data received. */
  SystemSensorData systemSensorData; /** The battery status received. */
  Semaphore frameDataSignal; /**< A signal used for synchronizing lowStateHandler() and waitForFrameData2(). */

#ifdef TARGET_BOOSTER
  booster::robot::b1::B1LocoClient client; /**< A client for activating custom mode again after emergency mode. */
  booster::robot::ChannelPublisher<booster_interface::msg::LowCmd> lowCmdPublisher; /**< Publisher for low level joint commands. */
  booster::robot::ChannelSubscriber<booster_interface::msg::LowState> lowStateSubscriber; /**< Subscriber for low level joint measurements. */
  booster::robot::ChannelSubscriber<booster_interface::msg::RobotStatusDdsMsg> robotStatusSubscriber; /**< Subscriber for the robot status. */
  booster::robot::RobotMode robotMode = booster::robot::RobotMode::kUnknown; /**< The current client mode of the robot. kUnknown equals emergency mode. */
  booster_interface::msg::LowCmd lowCmd; /**< The current low level joint command. */
  int keyInputHandle; /**< The input events from the keyboard of the robot. */
  size_t connectedJoints; /**< How many joints are connected? */
  bool waitingForHighLevelServices = false; /**< Waiting for high-level services to connect? */
  bool waitingForLowLevelServices = true; /**< Waiting for low-level services to connect? */
  bool waitingAnnounced = false; /**< Was announced that we are waiting for the robot to be ready? */
  bool unstiffAnnounced = false; /**< Was announced that we are waiting for the robot to be unstiffed? */
  bool waitingForModeSwitch = false; /**< Waiting for a mode switch to occur? */
  bool armPositionsOK = false; /**< Are the arms correctly rotated after starting? */
  unsigned armPositionWarning = 0; /**< Arms are not correctly rotated and was said at this timestamp. */
#endif

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theFrameInfo The representation updated.
   */
  void update(FrameInfo& theFrameInfo) override {SYNC; theFrameInfo.time = jointSensorData.timestamp;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJointSensorData The representation updated.
   */
#ifdef TARGET_BOOSTER
  void update(JointSensorData& theJointSensorData) override {SYNC; theJointSensorData = jointSensorData;}
#else
  void update(JointSensorData& theJointSensorData) override {SYNC; static_cast<JointAngles&>(theJointSensorData) = theJointRequest;}
#endif

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theKeyStates The representation updated.
   */
  void update(KeyStates& theKeyStates) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theRawInertialSensorData The representation updated.
   */
  void update(RawInertialSensorData& theRawInertialSensorData) override {SYNC; theRawInertialSensorData = rawInertialSensorData;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theSystemSensorData The representation updated.
   */
  void update(SystemSensorData& theSystemSensorData) override {SYNC; theSystemSensorData = systemSensorData;}

  /**
   * Get the name of the input event that is triggered by key presses.
   * @return The name of the input event.
   */
  std::string getInputEventName();

  /**
   * Process a low state message from the robot.
   * @param msg The message.
   */
  static void lowStateHandler(const void* msg);

  /**
   * Process robot status message from the robot.
   * @param msg The message.
   */
  static void robotStatusHandler(const void* msg);

#ifdef TARGET_BOOSTER
  /**
   * Process a low state message from the robot (called by static method).
   * @param lowState The low state message.
   */
  void lowStateHandler2(const booster_interface::msg::LowState& lowState);

  /**
   * Process a robot status message from the robot (called by static method).
   * @param robotStatus The low state message.
   */
  void robotStatusHandler2(const booster_interface::msg::RobotStatusDdsMsg& robotStatus);
#endif

  /** Waits for the next data to arrive from the Booster robot (called by static method). */
  void waitForFrameData2();

  /** Send requests to Booster robot (called by static method). */
  void finishFrame2();

public:
  BoosterProvider();
  ~BoosterProvider();

  /** Wait for the next data to arrive from the Booster robot. */
  static void waitForFrameData();

  /** Send requests to Booster robot. */
  static void finishFrame();
};
