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
#include "Platform/Thread.h"
#include "Platform/Time.h"
#ifdef TARGET_BOOSTER
#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>
#endif

MAKE_MODULE(BoosterProvider);

#ifndef TARGET_BOOSTER
thread_local
#endif
BoosterProvider* BoosterProvider::theInstance = nullptr;

#ifdef TARGET_BOOSTER
BoosterProvider::BoosterProvider()
  : lowCmdPublisher(booster::robot::b1::kTopicJointCtrl),
    lowStateSubscriber(booster::robot::b1::kTopicLowState, &BoosterProvider::lowStateHandler),
    robotStatusSubscriber("rt/device_gateway", &BoosterProvider::robotStatusHandler)
{
  theInstance = this;
  client.Init();
  lowCmdPublisher.InitChannel();
  lowStateSubscriber.InitChannel();
  robotStatusSubscriber.InitChannel();
  lowCmd.cmd_type(booster_interface::msg::SERIAL);
  lowCmd.motor_cmd(std::vector<booster_interface::msg::MotorCmd>(jointMapping.size()));
  connectedJoints = lowCmd.motor_cmd().size();
  keyInputHandle = open(getInputEventName().c_str(), O_NONBLOCK, O_RDONLY);
  //ASSERT(keyInputHandle != -1);
  Thread::sleep(10);
  if(!client.ChangeMode(booster::robot::RobotMode::kCustom))
  {
    robotMode = booster::robot::RobotMode::kCustom;
    waitingForLowLevelServices = false;
    Thread::sleep(10);
  }
  else
  {
    SystemCall::say("Waiting for high level services");
    waitingForHighLevelServices = true;
    Thread::sleep(100);
  }
#else
BoosterProvider::BoosterProvider()
{
  theInstance = this;
#endif
}

BoosterProvider::~BoosterProvider()
{
#ifdef TARGET_BOOSTER
  if(keyInputHandle != -1)
    close(keyInputHandle);
  robotStatusSubscriber.CloseChannel();
  lowStateSubscriber.CloseChannel();
  lowCmdPublisher.CloseChannel();
#endif
  theInstance = nullptr;
}

void BoosterProvider::update([[maybe_unused]] KeyStates& theKeyStates)
{
#ifdef TARGET_BOOSTER
  input_event inputEvent;
  if(keyInputHandle != -1 && ::read(keyInputHandle, &inputEvent, sizeof(inputEvent)) == sizeof(inputEvent) && inputEvent.type == EV_KEY)
    theKeyStates.pressed[inputEvent.code == 2 ? KeyStates::headFront
                         : inputEvent.code == 3 ? KeyStates::chest
                         : KeyStates::headRear] = static_cast<bool>(inputEvent.value);
#endif
}

std::string BoosterProvider::getInputEventName()
{
  File file("/proc/bus/input/devices", "r");
  char line[1000];
  while(!file.eof())
    if(std::string(file.readLine(line, sizeof(line))).find("Name=\"HID 5131:2019\"") != std::string::npos)
    {
      while(!file.eof() && std::string(file.readLine(line, sizeof(line))).find("Handlers=") == std::string::npos);
      if(!file.eof() && std::string(line).find("Handlers=sysrq kbd") != std::string::npos)
      {
        const size_t pos = std::string(line).find("event");
        if(pos != std::string::npos)
        {
          char* p = line + strlen(line);
          while(isspace(p[-1]))
            --p;
          *p = 0;
          return std::string("/dev/input/") + (line + pos);
        }
      }
    }
  return "";
}

void BoosterProvider::lowStateHandler([[maybe_unused]] const void* msg)
{
#ifdef TARGET_BOOSTER
  if(theInstance)
    theInstance->lowStateHandler2(*reinterpret_cast<const booster_interface::msg::LowState*>(msg));
#endif
}

void BoosterProvider::robotStatusHandler([[maybe_unused]] const void* msg)
{
#ifdef TARGET_BOOSTER
  if(theInstance)
    theInstance->robotStatusHandler2(*reinterpret_cast<const booster_interface::msg::RobotStatusDdsMsg*>(msg));
#endif
}

#ifdef TARGET_BOOSTER
void BoosterProvider::lowStateHandler2(const booster_interface::msg::LowState& lowState)
{
  ASSERT(lowState.motor_state_serial().size() == jointMapping.size()); // TODO check whether this works for K1
  SYNC;
  for(size_t joint : {Joints::lWristYaw, Joints::lHand, Joints::rWristYaw, Joints::rHand})
  {
    jointSensorData.angles[joint] = 0_deg;
    jointSensorData.currents[joint] = 0;
    jointSensorData.temperatures[joint] = 0;
    jointSensorData.status[joint] = JointSensorData::regular;
  }

  for(size_t joint = 0; joint < lowState.motor_state_serial().size(); ++joint)
  {
    const size_t index = jointMapping[joint];
    ASSERT(index != Joints::numOfJoints);
    jointSensorData.angles[index] = lowState.motor_state_serial()[joint].q();
    jointSensorData.velocity[index] = lowState.motor_state_serial()[joint].dq();
    jointSensorData.currents[index] = static_cast<short>(lowState.motor_state_parallel()[joint].tau_est() * 100.f); // This is torque (1/100 Nm), not current
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
        SystemCall::playSound("siren", true);
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

  rawInertialSensorData.gyro = {lowState.imu_state().gyro()[0], lowState.imu_state().gyro()[1], lowState.imu_state().gyro()[2]};
  rawInertialSensorData.acc = {lowState.imu_state().acc()[0], lowState.imu_state().acc()[1], lowState.imu_state().acc()[2]};
  rawInertialSensorData.angle = {lowState.imu_state().rpy()[0], lowState.imu_state().rpy()[1], lowState.imu_state().rpy()[2]};

  jointSensorData.timestamp = Time::getRealSystemTime();
  frameDataSignal.post();
}

void BoosterProvider::robotStatusHandler2(const booster_interface::msg::RobotStatusDdsMsg& robotStatus)
{
  SYNC;

  const std::vector<booster_interface::msg::RobotDdsBatteryStatus>& batteryStatuses = robotStatus.battery_vec();
  if(!batteryStatuses.empty())
  {
    systemSensorData.batteryLevel = batteryStatuses.front().soc() * 0.01f;
    systemSensorData.batteryCharging = false;
  }

  connectedJoints = 0;
  for(const booster_interface::msg::RobotDdsJointStatus& joint : robotStatus.joint_vec())
    if(joint.is_connected())
      ++connectedJoints;
}
#endif

void BoosterProvider::waitForFrameData()
{
  if(theInstance)
    theInstance->waitForFrameData2();
}

void BoosterProvider::waitForFrameData2()
{
#ifdef TARGET_BOOSTER
  if(waitingForHighLevelServices)
  {
    if(Global::getSettings().robotType == Settings::k1 && !client.ChangeMode(booster::robot::RobotMode::kPrepare))
      robotMode = booster::robot::RobotMode::kPrepare;
    if(client.ChangeMode(booster::robot::RobotMode::kCustom))
    {
      if(!client.ChangeMode(booster::robot::RobotMode::kDamping))
        robotMode = booster::robot::RobotMode::kDamping;
      Thread::sleep(100);
      return;
    }
    else
    {
      waitingForHighLevelServices = false;
      waitingForLowLevelServices = false;
      robotMode = booster::robot::RobotMode::kCustom;
    }
  }

  bool lowStateReceived = false;
  if(!frameDataSignal.wait(maxDelayForFrameData))
  {
    if(!theFrameInfo.time && !waitingAnnounced)
    {
      SystemCall::say("Waiting for low level services");
      waitingAnnounced = true;
      robotMode = booster::robot::RobotMode::kUnknown;
    }
  }
  else
  {
    while(frameDataSignal.tryWait());
    lowStateReceived = true;
  }

  size_t connectedJoints;
  {
    SYNC; // allConnected is set in another thread
    connectedJoints = this->connectedJoints;
  }

  bool canLeaveEmergencyMode = false;

  if((!connectedJoints || !lowStateReceived) && robotMode != booster::robot::RobotMode::kUnknown)
  {
    SystemCall::say("Emergency", true);
    robotMode = booster::robot::RobotMode::kUnknown;
    unstiffAnnounced = false;
  }
  else if(connectedJoints == lowCmd.motor_cmd().size() && lowStateReceived && robotMode == booster::robot::RobotMode::kUnknown)
  {
    // Only leave emergency mode if the motion request is play dead and the requested stiffness is zero for all joints.
    canLeaveEmergencyMode = theMotionRequest.motion == MotionRequest::playDead;
    for(size_t joint = 0; joint < lowCmd.motor_cmd().size(); ++joint)
    {
      const size_t index = jointMapping[joint];
      ASSERT(index != Joints::numOfJoints);
      if(theJointRequest.angles[index] != SensorData::off && theJointRequest.stiffnessData.stiffnesses[index] > 0)
        canLeaveEmergencyMode = false;
    }
    if(!canLeaveEmergencyMode && !unstiffAnnounced)
    {
      SystemCall::say("Please manually unstiff the robot");
      unstiffAnnounced = true;
    }
  }

  const booster::robot::RobotMode targetMode = theMotionRequest.motion == MotionRequest::playDead && !waitingForLowLevelServices
                                               ? booster::robot::RobotMode::kDamping : booster::robot::RobotMode::kCustom;

  if(robotMode != targetMode && (robotMode != booster::robot::RobotMode::kUnknown || canLeaveEmergencyMode))
  {
    if(Global::getSettings().robotType == Settings::k1 && targetMode == booster::robot::RobotMode::kCustom && !client.ChangeMode(booster::robot::RobotMode::kPrepare))
      robotMode = booster::robot::RobotMode::kPrepare;
    if(!client.ChangeMode(targetMode))
    {
      if(robotMode == booster::robot::RobotMode::kUnknown)
        SystemCall::say("Body ready", true);
      robotMode = targetMode;
      waitingForModeSwitch = false;
      waitingForLowLevelServices = false;
    }
    else
    {
      if(!client.ChangeMode(booster::robot::RobotMode::kDamping)) // Fail-Safe. Robot is in an unknown state
        robotMode = booster::robot::RobotMode::kDamping;
      if(!waitingForModeSwitch && robotMode != booster::robot::RobotMode::kUnknown)
      {
        SystemCall::say("Switching robot mode failed", true);
        waitingForModeSwitch = true;
      }
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
  DEBUG_RESPONSE("module:BoosterProvider:segfault")* static_cast<volatile char*>(nullptr) = 0;

#ifdef TARGET_BOOSTER
  ASSERT(lowCmd.motor_cmd().size() == jointMapping.size());
  ASSERT(lowCmd.motor_cmd().size() == controlParameters.size());
  for(size_t joint = 0; joint < lowCmd.motor_cmd().size(); ++joint)
  {
    booster_interface::msg::MotorCmd& motorCmd = lowCmd.motor_cmd()[joint];
    const size_t index = jointMapping[joint];
    ASSERT(index != Joints::numOfJoints);
    if(robotMode == booster::robot::RobotMode::kUnknown
       || theJointRequest.angles[index] == SensorData::off
       || theJointRequest.stiffnessData.stiffnesses[index] <= 0)
    {
      motorCmd.q(jointSensorData.angles[index]);
      motorCmd.kp(0.f);
      motorCmd.kd(0.f);
      motorCmd.tau(0.f);
    }
    else
    {
      motorCmd.q(theJointRequest.angles[index] + theJointCalibration.offsets[index]);
      const float stiffness = Rangef::ZeroOneRange().limit(theJointRequest.stiffnessData.stiffnesses[index] * 0.01f);
      motorCmd.kp(controlParameters[joint].p * stiffness);
      motorCmd.kd(controlParameters[joint].d * stiffness);
      if(setTorque)
        motorCmd.tau(Rangef(-controlParameters[joint].maxTorque * stiffness, controlParameters[joint].maxTorque * stiffness).limit(theModifiedJointRequest.angles[index] * controlParameters[joint].p * stiffness));
      else
        motorCmd.tau(0);
    }
    motorCmd.weight(1.f);
  }

  VERIFY(lowCmdPublisher.Write(&lowCmd));
#endif
}
