/**
 * @file BoosterProvider.cpp
 *
 * This file implements a module that connects to the Booster robot.
 * In playDead, the damping mode is commanded. In any other case,
 * the custom mode is used. When waiting for a connection to the
 * robot, the custom mode is requested as well, because the robot
 * seems to accept the damping mode when it is not completely ready.
 *
 * @author Thomas Röfer
 */

#include "BoosterProvider.h"
#include "Framework/Settings.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#ifdef TARGET_ROBOT
#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>
#include <booster/robot/device/light/light_control_client.hpp>
#endif

MAKE_MODULE(BoosterProvider);

// Use instances initialized in main function
#ifdef TARGET_ROBOT
namespace Main
{
  extern booster::robot::b1::B1LocoClient client;
  extern booster::robot::light::LightControlClient ledControl;
}
using namespace Main;
#endif

thread_local BoosterProvider* BoosterProvider::theInstance = nullptr;

BoosterProvider::BoosterProvider()
#ifdef TARGET_ROBOT
  : lowCmdPublisher(booster::robot::b1::kTopicJointCtrl),
    buttonEventSubscriber("rt/button_even2", [this](const void* msg)
      {buttonEventHandler(*reinterpret_cast<const booster_interface::msg::ButtonEventMsg*>(msg));}),
    lowStateSubscriber(booster::robot::b1::kTopicLowState, [this](const void* msg)
      {lowStateHandler(*reinterpret_cast<const booster_interface::msg::LowState*>(msg));}),
    remoteControllerStateSubscriber("rt/remote_controller_stat2", [this](const void* msg)
      {remoteControllerStateHandler(*reinterpret_cast<const booster_interface::msg::RemoteControllerState*>(msg));}),
    robotStatusSubscriber("rt/device_gateway", [this](const void* msg)
      {robotStatusHandler(*reinterpret_cast<const booster_interface::msg::RobotStatusDdsMsg*>(msg));})
#endif
{
  theInstance = this;
#ifdef TARGET_ROBOT
  lowCmdPublisher.InitChannel();
  lowStateSubscriber.InitChannel();
  remoteControllerStateSubscriber.InitChannel();
  robotStatusSubscriber.InitChannel();
  lowCmd.cmd_type(booster_interface::msg::SERIAL);
  lowCmd.motor_cmd(std::vector<booster_interface::msg::MotorCmd>(jointMapping.size()));
  systemSensorData.emergencyMode = true;
  connectedJoints = lowCmd.motor_cmd().size();
  if(Global::getSettings().robotType == Settings::k1)
  {
    buttonEventSubscriber.InitChannel();

    // Force to set LEDs at least once.
    ledState.r = 1;
  }
  start(this, &BoosterProvider::dampingModeHandler);

  char zoneTemperaturePath[] = "/sys/devices/virtual/thermal/thermal_zone0/temp";
  for(size_t i = 0; i < numOfThermalZones; ++i)
  {
    zoneTemperatureFiles[i] = ::open(zoneTemperaturePath, O_RDONLY);
    ASSERT(zoneTemperatureFiles[i] >= 0);
    ++zoneTemperaturePath[41];
  }
#endif
}

BoosterProvider::~BoosterProvider()
{
#ifdef TARGET_ROBOT
  stop();
  for(size_t i = 0; i < numOfThermalZones; ++i)
    close(zoneTemperatureFiles[i]);
  if(Global::getSettings().robotType == Settings::k1)
    buttonEventSubscriber.CloseChannel();
  robotStatusSubscriber.CloseChannel();
  remoteControllerStateSubscriber.CloseChannel();
  lowStateSubscriber.CloseChannel();
  lowCmdPublisher.CloseChannel();
#endif
  theInstance = nullptr;
}

void BoosterProvider::update([[maybe_unused]] KeyStates& theKeyStates)
{
#ifdef TARGET_ROBOT
  {
    SYNC;
    theKeyStates = buttonKeyStates;
  }

  FOREACH_ENUM(KeyStates::Key, key)
    theKeyStates.pressed[key] |= joystickKeyStates.pressed[key];

  // Overlay provided key states with press of F1 in emergency mode to force switch to playDead.
  if(systemSensorData.emergencyMode && (!theMotionInfo.isMotion(MotionPhase::playDead) || theMotionRequest.motion != MotionRequest::playDead))
    theKeyStates.pressed[KeyStates::f1] = true;
#endif
}

void BoosterProvider::update(SystemSensorData& theSystemSensorData)
{
#ifdef TARGET_ROBOT
  if(theFrameInfo.getTimeSince(timeWhenTemperatureRead) >= timeBetweenTemperatureUpdates)
  {
    theSystemSensorData.cpuTemperature = readTemperature(0);
    theSystemSensorData.gpuTemperature = readTemperature(1);
    timeWhenTemperatureRead = theFrameInfo.time;
  }
#endif

  SYNC;
  theSystemSensorData.batteryLevel = systemSensorData.batteryLevel;
  theSystemSensorData.emergencyMode = systemSensorData.emergencyMode;
}

#ifdef TARGET_ROBOT
void BoosterProvider::buttonEventHandler(const booster_interface::msg::ButtonEventMsg& buttonEvent)
{
  const booster_interface::msg::ButtonEventType event = buttonEvent.event();
  if(event == booster_interface::msg::PRESS_DOWN || event == booster_interface::msg::PRESS_UP)
  {
    const int button = buttonEvent.button();
    SYNC;
    buttonKeyStates.pressed[button == 2 ? KeyStates::walk
                                   : button == 1 ? KeyStates::stand
                                   : KeyStates::f1] = event == booster_interface::msg::PRESS_DOWN;
  }
}

void BoosterProvider::lowStateHandler(const booster_interface::msg::LowState& lowState)
{
  // During termination, the low state might be empty
  if(lowState.motor_state_serial().size() != jointMapping.size())
    return;

  SYNC;
  for(size_t joint = 0; joint < lowState.motor_state_serial().size(); ++joint)
  {
    const size_t index = jointMapping[joint];
    ASSERT(index != Joints::numOfJoints); // This is not reported correctly, because this thread has no BH_TRACE_INIT
    jointSensorData.angles[index] = lowState.motor_state_serial()[joint].q();
    jointSensorData.velocity[index] = lowState.motor_state_serial()[joint].dq();
    jointSensorData.torque[index] = lowState.motor_state_parallel()[joint].tau_est();
    const unsigned char temperature = lowState.motor_state_parallel()[joint].temperature();
    jointSensorData.temperatures[index] = temperature;
    jointSensorData.status[index] = temperature < minHot ?  JointSensorData::regular
                                    : temperature < minVeryHot ? JointSensorData::hot
                                    : temperature < minCriticallyHot ? JointSensorData::veryHot
                                    : JointSensorData::criticallyHot;
  }

  if(!armPositionsOK)
  {
    if(!shoulderPitchRangeAtStart.isInside(jointSensorData.angles[Joints::lShoulderPitch]) || !shoulderPitchRangeAtStart.isInside(jointSensorData.angles[Joints::rShoulderPitch]))
    {
      if(theFrameInfo.getTimeSince(armPositionWarning) > armsWarningTime)
      {
        armPositionWarning = theFrameInfo.time;
        SystemCall::playSound("siren.wav", true);
        SystemCall::say("Wrong Arm Position");
        if(!shoulderPitchRangeAtStart.isInside(jointSensorData.angles[Joints::lShoulderPitch]))
          SystemCall::say("Left Arm");
        if(!shoulderPitchRangeAtStart.isInside(jointSensorData.angles[Joints::rShoulderPitch]))
          SystemCall::say("Right Arm");
      }
    }
    else
    {
      armPositionsOK = true;
      if(armPositionWarning != 0)
        SystemCall::say("Arms are correct");
    }
  }

  inertialSensorData.gyro = {lowState.imu_state().gyro()[0], lowState.imu_state().gyro()[1], lowState.imu_state().gyro()[2]};
  inertialSensorData.acc = {lowState.imu_state().acc()[0], lowState.imu_state().acc()[1], lowState.imu_state().acc()[2]};
  inertialSensorData.angle = {lowState.imu_state().rpy()[0], lowState.imu_state().rpy()[1], lowState.imu_state().rpy()[2]};

  jointSensorData.timestamp = Time::getRealSystemTime();
  frameDataSignal.post();
}

void BoosterProvider::remoteControllerStateHandler(const booster_interface::msg::RemoteControllerState& remoteControllerState)
{
  SYNC;
  if(remoteControllerState.event() == booster::robot::b1::REMOVE)
    joystickState = JoystickState();
  else
  {
    joystickState.axes[0] = -remoteControllerState.lx();
    joystickState.axes[1] = -remoteControllerState.ly();
    joystickState.axes[2] = -remoteControllerState.rx();
    joystickState.axes[3] = -remoteControllerState.ry();
    joystickState.axes[6] = (remoteControllerState.hat_l() || remoteControllerState.hat_lu() || remoteControllerState.hat_ld() ? 1.f : 0.f)
                          - (remoteControllerState.hat_r() || remoteControllerState.hat_ru() || remoteControllerState.hat_rd() ? 1.f : 0.f);
    joystickState.axes[7] = (remoteControllerState.hat_u() || remoteControllerState.hat_lu() || remoteControllerState.hat_ru() ? 1.f : 0.f)
                          - (remoteControllerState.hat_d() || remoteControllerState.hat_ld() || remoteControllerState.hat_rd() ? 1.f : 0.f);
    joystickState.buttons = (remoteControllerState.x() ? 1 : 0)       // X
                          | (remoteControllerState.a() ? 2 : 0)       // A
                          | (remoteControllerState.b() ? 4 : 0)       // B
                          | (remoteControllerState.y() ? 8 : 0)       // Y
                          | (remoteControllerState.lb() ? 16 : 0)     // left shoulder
                          | (remoteControllerState.rb() ? 32 : 0)     // right shoulder
                          | (remoteControllerState.lt() ? 64 : 0)     // left trigger
                          | (remoteControllerState.rt() ? 128 : 0)    // right trigger
                          | (remoteControllerState.back() ? 256 : 0)  // -
                          | (remoteControllerState.start() ? 512 : 0) // +
                          | (remoteControllerState.ls() ? 1024 : 0)   // left stick
                          | (remoteControllerState.rs() ? 2048 : 0);  // right right
    joystickState.valid = true;
  }
}

void BoosterProvider::update(JoystickState& theJoystickState)
{
  SYNC;
  theJoystickState = joystickState;
  joystickKeyStates.pressed.fill(false);
  if(theJoystickState.pressed(rtButtonCode))
  {
    for(const ButtonMapping& buttonMapping : buttonMappings)
      joystickKeyStates.pressed[buttonMapping.key] |= theJoystickState.pressed(buttonMapping.code);
  }
}

void BoosterProvider::update(DampingState& theDampingState)
{
  theDampingState.isDamping = robotMode != booster::robot::RobotMode::kCustom;
}

void BoosterProvider::robotStatusHandler(const booster_interface::msg::RobotStatusDdsMsg& robotStatus)
{
  SYNC;

  const std::vector<booster_interface::msg::RobotDdsBatteryStatus>& batteryStatuses = robotStatus.battery_vec();
  if(!batteryStatuses.empty())
    systemSensorData.batteryLevel = batteryStatuses.front().soc() * 0.01f;

  connectedJoints = 0;
  for(const booster_interface::msg::RobotDdsJointStatus& joint : robotStatus.joint_vec())
    if(joint.is_connected())
      ++connectedJoints;
}

void BoosterProvider::dampingModeHandler()
{
  booster::robot::b1::GetModeResponse modeResponse;
  int failedGetMode = 0;
  while(isRunning())
  {
    sleep(10);
    if(client.GetMode(modeResponse))
    {
      SYNC;
      if(++failedGetMode >= failedGetModeThreshold)
        robotMode = booster::robot::RobotMode::kUnknown;
    }
    else
    {
      SYNC;
      robotMode = modeResponse.mode_;
      failedGetMode = 0;
    }
  }
}

float BoosterProvider::readTemperature(const size_t zone) const
{
  char buffer[8];
  ssize_t length = ::pread(zoneTemperatureFiles[zone], buffer, 8, 0);
  if(length > 1)
  {
    ASSERT(buffer[length - 1] == '\n');
    return static_cast<float>(std::atoi(buffer)) / 1000.f;
  }
  else
    return SensorData::off;
}
#else
void BoosterProvider::update(JoystickState& theJoystickState)
{
  SYNC;
  theJoystickState = joystickState;
}

void BoosterProvider::update(DampingState& theDampingState)
{
  theDampingState.isDamping = false;
}
#endif

void BoosterProvider::waitForFrameData()
{
  if(theInstance)
    theInstance->waitForFrameData2();
}

void BoosterProvider::waitForFrameData2()
{
#ifdef TARGET_ROBOT
  bool lowStateReceived = false;
  if(frameDataSignal.wait(maxDelayForFrameData))
  {
    while(frameDataSignal.tryWait());
    lowStateReceived = true;
  }

  size_t connectedJoints;
  booster::robot::RobotMode robotMode;
  {
    SYNC; // both values are set in other threads
    connectedJoints = this->connectedJoints;
    robotMode = this->robotMode;
  }

  bool canLeaveEmergencyMode = false;

  const bool emergencyDetected = !connectedJoints || !lowStateReceived
                                 || robotMode != booster::robot::RobotMode::kCustom;
  if(!systemSensorData.emergencyMode && emergencyDetected)
  {
    if(!connectedJoints)
      SystemCall::say("Emergency, no joints", true);
    else if(!lowStateReceived)
      SystemCall::say("Emergency, no low state", true);
    else if(robotMode == booster::robot::RobotMode::kUnknown)
      SystemCall::say("Emergency, no robot mode", true);
    else
      SystemCall::say("Emergency", true);
    systemSensorData.emergencyMode = true;
  }
  else if(systemSensorData.emergencyMode && !emergencyDetected && connectedJoints == lowCmd.motor_cmd().size())
  {
    // Only leave emergency mode if the motion request is play dead and the requested stiffness is zero for all joints.
    canLeaveEmergencyMode = theMotionInfo.isMotion(MotionPhase::playDead) && theMotionRequest.motion == MotionRequest::playDead;
    for(size_t joint = 0; joint < lowCmd.motor_cmd().size(); ++joint)
    {
      const size_t index = jointMapping[joint];
      ASSERT(index != Joints::numOfJoints);
      if(theJointRequest.angles[index] != SensorData::off && theJointRequest.stiffnessData.stiffnesses[index] > 0)
        canLeaveEmergencyMode = false;
    }
    if(canLeaveEmergencyMode)
    {
      SystemCall::say("Body ready", true);
      systemSensorData.emergencyMode = false;
    }
  }
#else
  Thread::yield();
#endif
}

void BoosterProvider::finishFrame()
{
  if(theInstance)
    theInstance->finishFrame2();
}

void BoosterProvider::finishFrame2()
{
  DEBUG_RESPONSE("module:BoosterProvider:segfault") *static_cast<volatile char*>(nullptr) = 0;

#ifdef TARGET_ROBOT
  ASSERT(lowCmd.motor_cmd().size() == jointMapping.size());
  FOREACH_ENUM(JointStiffnessMapping::Type, type)
    ASSERT(lowCmd.motor_cmd().size() <= theMotorParameters.controlParameters[type].size());
  for(size_t joint = 0; joint < lowCmd.motor_cmd().size(); ++joint)
  {
    booster_interface::msg::MotorCmd& motorCmd = lowCmd.motor_cmd()[joint];
    const size_t index = jointMapping[joint];
    ASSERT(index != Joints::numOfJoints);
    if(systemSensorData.emergencyMode
       || theJointRequest.angles[index] == SensorData::off
       || theJointRequest.stiffnessData.stiffnesses[index] <= 0)
    {
      motorCmd.q(jointSensorData.angles[index]);
      motorCmd.kp(0.f);
      motorCmd.kd(theMotorParameters.dDamping[index]);
      motorCmd.tau(0.f);
    }
    else if(!theJointRequest.isTorqueControl || index == Joints::headYaw || index == Joints::headPitch)
    {
      motorCmd.q(theJointRequest.angles[index] + theJointCalibration.offsets[index]);
      const float stiffness = Rangef::ZeroOneRange().limit(theJointRequest.stiffnessData.stiffnesses[index] * 0.01f);
      motorCmd.kp(theMotorParameters.controlParameters[theJointRequest.jointStiffnessMapping][index].p * stiffness);
      motorCmd.kd(theMotorParameters.controlParameters[theJointRequest.jointStiffnessMapping][index].d * stiffness);
      motorCmd.tau(0.f);
      if(index == Joints::headPitch)
      {
        const Angle headDiff = std::max(0.f, theJointRequest.angles[index] + theJointCalibration.offsets[index] - startHeadPitchTorque);
        motorCmd.tau(Rangef(-theMotorParameters.controlParameters[theJointRequest.jointStiffnessMapping][index].maxTorque * stiffness, theMotorParameters.controlParameters[theJointRequest.jointStiffnessMapping][index].maxTorque * stiffness).limit(headDiff * headPitchTorqueDiff * theMotorParameters.controlParameters[theJointRequest.jointStiffnessMapping][index].p * stiffness));
      }
    }
    else
    {
      motorCmd.q(jointSensorData.angles[index]);
      const float stiffness = Rangef::ZeroOneRange().limit(theJointRequest.stiffnessData.stiffnesses[index] * 0.01f);
      motorCmd.kp(0.f);
      motorCmd.kd(theMotorParameters.controlParameters[theJointRequest.jointStiffnessMapping][index].d * stiffness);
      motorCmd.tau(Rangef(-theMotorParameters.controlParameters[theJointRequest.jointStiffnessMapping][index].maxTorque, theMotorParameters.controlParameters[theJointRequest.jointStiffnessMapping][index].maxTorque).limit((theJointRequest.angles[index] + theJointCalibration.offsets[index] - jointSensorData.angles[index]) * theMotorParameters.controlParameters[theJointRequest.jointStiffnessMapping][index].p * stiffness));
    }
    motorCmd.weight(1.f);
  }

  VERIFY(lowCmdPublisher.Write(&lowCmd));

  if(Global::getSettings().robotType == Settings::k1)
  {
    if(!systemSensorData.emergencyMode)
    {
      const bool on = (jointSensorData.timestamp / 400 & 1) == 1;
      const bool fastOn = (jointSensorData.timestamp / 80 & 1) == 0;
      bool changed = false;
      FOREACH_ENUM(LEDRequest::LED, led)
      {
        const LEDRequest::State state = theLEDRequest.ledStates[led];
        const uint8_t newValue = state == LEDRequest::on
        || (state == LEDRequest::blinking && on)
        || (state == LEDRequest::fastBlinking && fastOn)
        ? 255
        : state == LEDRequest::half
        || (state == LEDRequest::halfBlinking && on)
        || (state == LEDRequest::halfFastBlinking && fastOn)? 128 : 0;
        uint8_t& value = (&ledState.r)[led];
        if(newValue != value)
        {
          value = newValue;
          changed = true;
        }
      }
      if(changed)
        VERIFY(!ledControl.SetLEDLightColor(ledState.r, ledState.g, ledState.b));
    }
    else if(ledState.r != 1)
    {
      ledControl.StopLEDLightControl();
      ledState.r = 1;
    }
  }
#endif
}
