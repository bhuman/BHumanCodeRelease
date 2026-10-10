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
#include "ImageProcessing/PixelTypes.h"
#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/MotorParameters.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/JoystickState.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/MotionControl/DampingState.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Streaming/EnumIndexedArray.h"

#ifdef TARGET_ROBOT
#include <booster/idl/b1/ButtonEvent.h>
#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/RemoteControllerState.h>
#include <booster/idl/b1/RobotStatusDdsMsg.h>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#endif

MODULE(BoosterProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(JointCalibration),
  USES(JointRequest),
  USES(JointSensorData),
  REQUIRES(LEDRequest),
  USES(MotionInfo),
  REQUIRES(MotionRequest),
  REQUIRES(MotorParameters),
  PROVIDES(DampingState),
  PROVIDES(FrameInfo),
  PROVIDES(JointSensorData),
  PROVIDES(JoystickState),
  PROVIDES(KeyStates),
  PROVIDES(InertialSensorData),
  PROVIDES(SystemSensorData),
  LOADS_PARAMETERS(
  {
    STREAMABLE(ButtonMapping,
    {,
      (int) code, /**< The input event code for the button. */
      (KeyStates::Key) key, /**< The key the code is mapped to. */
    }),

    (unsigned char) minHot, /**< Minimum temperature for status hot (in °C). */
    (unsigned char) minVeryHot, /**< Minimum temperature for status very hot (in °C). */
    (unsigned char) minCriticallyHot, /**< Minimum temperature for status critically hot (in °C). */
    (int) maxDelayForFrameData, /**< The maximum time waited for new frame data. */
    (std::vector<Joints::Joint>) jointMapping, /**< The mapping from Booster joints indices to B-Human joint indices. */
    (Rangea) shoulderPitchRangeAtStart, /**< At start the shoulder pitches should be insides this range. */
    (int) armsWarningTime, /**< Warn about the arm positions after this much time passed. */
    (int) rtButtonCode, /**< Button code for RT button. */
    (std::vector<ButtonMapping>) buttonMappings, /**< Mappings from key codes to keys on T1 robots (ignored on K1). */
    (int) failedGetModeThreshold, /**< The number of times GetMode failed before switching to emergency mode. */
    (Angle) startHeadPitchTorque, /**< Start applying torque for the head pitch due to boosters joint clipping. */
    (float) headPitchTorqueDiff, /**< Use diff of head pitch target and startHeadPitchTorque times this factor to calculate torque. */
    (int)(5000) timeBetweenTemperatureUpdates, /**< Time between reading the CPU temperature (in ms). */
  }),
});

class BoosterProvider : public BoosterProviderBase, public Thread
{
  DECLARE_SYNC; /**< Data is received in a different thread, so synchronize access. */
  thread_local static BoosterProvider* theInstance; /**< The only instance of this module. */
  JointSensorData jointSensorData; /**< The joint sensor data received. */
  JoystickState joystickState; /**< The joystick state received. */
  InertialSensorData inertialSensorData; /**< The inertial sensor data received. */
  SystemSensorData systemSensorData; /** The system status received. */
  KeyStates buttonKeyStates; /** The key states received if button event messages are available. */
  KeyStates joystickKeyStates; /** The simulated button key states with a joystick controller. */
  Semaphore frameDataSignal; /**< A signal used for synchronizing lowStateHandler() and waitForFrameData2(). */

#ifdef TARGET_ROBOT
  booster::robot::ChannelPublisher<booster_interface::msg::LowCmd> lowCmdPublisher; /**< Publisher for low level joint commands. */
  booster::robot::ChannelSubscriber<booster_interface::msg::ButtonEventMsg> buttonEventSubscriber; /**< Subscriber for button events. */
  booster::robot::ChannelSubscriber<booster_interface::msg::LowState> lowStateSubscriber; /**< Subscriber for low level joint measurements. */
  booster::robot::ChannelSubscriber<booster_interface::msg::RemoteControllerState> remoteControllerStateSubscriber; /**< Subscriber for the remote controller state. */
  booster::robot::ChannelSubscriber<booster_interface::msg::RobotStatusDdsMsg> robotStatusSubscriber; /**< Subscriber for the robot status. */
  booster::robot::RobotMode robotMode = booster::robot::RobotMode::kCustom; /**< The current client mode of the robot. */
  booster_interface::msg::LowCmd lowCmd; /**< The current low level joint command. */
  size_t connectedJoints; /**< How many joints are connected? */
  bool armPositionsOK = false; /**< Are the arms correctly rotated after starting? */
  unsigned armPositionWarning = 0; /**< Arms are not correctly rotated and was said at this timestamp. */
  PixelTypes::RGBPixel ledState; /**< The current state of the leds. **/
  unsigned timeWhenTemperatureRead = 0; /**< The last time the temperature was read. */
  static constexpr size_t numOfThermalZones = 2; /**< The number of thermal zones read in the CPU/GPU. */
  int zoneTemperatureFiles[numOfThermalZones]; /**< The file descriptors where the thermal zone temperatures can be read from. */
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
#ifdef TARGET_ROBOT
  void update(JointSensorData& theJointSensorData) override {SYNC; theJointSensorData = jointSensorData;}
#else
  void update(JointSensorData& theJointSensorData) override {SYNC; static_cast<JointAngles&>(theJointSensorData) = theJointRequest;}
#endif

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJoystickState The representation updated.
   */
  void update(JoystickState& theJoystickState) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theDampingState The representation updated.
   */
  void update(DampingState& theDampingState) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theKeyStates The representation updated.
   */
  void update(KeyStates& theKeyStates) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theInertialSensorData The representation updated.
   */
  void update(InertialSensorData& theInertialSensorData) override {SYNC; theInertialSensorData = inertialSensorData;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theSystemSensorData The representation updated.
   */
  void update(SystemSensorData& theSystemSensorData) override;

#ifdef TARGET_ROBOT
  /**
   * Process a button event message from the robot.
   * @param buttonEvent The button event message.
   */
  void buttonEventHandler(const booster_interface::msg::ButtonEventMsg& buttonEvent);

  /**
   * Process a low state message from the robot.
   * @param lowState The low state message.
   */
  void lowStateHandler(const booster_interface::msg::LowState& lowState);

  /**
   * Process a remote controller state message from the robot.
   * @param remoteControllerState The remote controller state message.
   */
  void remoteControllerStateHandler(const booster_interface::msg::RemoteControllerState& remoteControllerState);

  /**
   * Process a robot status message from the robot.
   * @param robotStatus The robot status message.
   */
  void robotStatusHandler(const booster_interface::msg::RobotStatusDdsMsg& robotStatus);

  /** A separate thread that continuously polls the current mode. */
  void dampingModeHandler();

  /**
   * Reads the temperatures of the thermal zone.
   * @param zone The index of the thermal zone.
   * @return The temperature of the thermal zone.
   */
  float readTemperature(const size_t zone) const;
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
