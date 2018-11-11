/**
 * @file Modules/Infrastructure/NaoProvider.cpp
 * The file declares a module that provides information from the Nao via DCM.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "NaoProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"

MAKE_MODULE(NaoProvider, motionInfrastructure)

#ifdef TARGET_ROBOT

#include "Platform/Time.h"
#include "Platform/File.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Settings.h"

#include "libbhuman/bhuman.h"

#include <cstdio>
#include <cstring>
#include <algorithm>

thread_local NaoProvider* NaoProvider::theInstance = nullptr;

NaoProvider::NaoProvider()
{
  NaoProvider::theInstance = this;
  memset(&gameControlData, 0, sizeof(gameControlData));

  for(int i = 0; i < Joints::numOfJoints; ++i)
    clippedLastFrame[i] = SensorData::off;
}

NaoProvider::~NaoProvider()
{
  NaoProvider::theInstance = nullptr;
}

void NaoProvider::finishFrame()
{
  if(theInstance)
    theInstance->send();
}

void NaoProvider::waitForFrameData()
{
  DEBUG_RESPONSE_ONCE("module:NaoProvider:robotName")
  {
    if(Global::getSettings().headName == Global::getSettings().bodyName)
      OUTPUT_TEXT("Hi, I am " << Global::getSettings().headName << ".");
    else
      OUTPUT_TEXT("Hi, I am " << Global::getSettings().headName << " (using " << Global::getSettings().bodyName << "'s body).");

    OUTPUT(idRobotname, bin, Global::getSettings().headName << Global::getSettings().bodyName << Global::getSettings().location);
  }

  if(theInstance)
    theInstance->naoBody.wait();
}

void NaoProvider::send()
{
  DEBUG_RESPONSE("module:NaoProvider:lag100") Thread::sleep(100);
  DEBUG_RESPONSE("module:NaoProvider:lag200") Thread::sleep(200);
  DEBUG_RESPONSE("module:NaoProvider:lag300") Thread::sleep(300);
  DEBUG_RESPONSE("module:NaoProvider:lag1000") Thread::sleep(1000);
  DEBUG_RESPONSE("module:NaoProvider:lag3000") Thread::sleep(3000);
  DEBUG_RESPONSE("module:NaoProvider:lag6000") Thread::sleep(6000);
  DEBUG_RESPONSE("module:NaoProvider:segfault") *static_cast<volatile char*>(nullptr) = 0;

  DEBUG_RESPONSE("module:NaoProvider:clippingInfo")
  {
    for(int i = 0; i < Joints::numOfJoints; ++i)
    {
      if(i == Joints::rHipYawPitch) // missing on Nao
        ++i;

      if(theJointRequest.angles[i] != SensorData::off)
      {
        if(theJointRequest.angles[i] > theJointLimits.limits[i].max)
        {
          if(clippedLastFrame[i] != theJointLimits.limits[i].max)
          {
            char tmp[64];
            sprintf(tmp, "warning: clipped joint %s at %.03f, requested %.03f.", TypeRegistry::getEnumName(static_cast<Joints::Joint>(i)), theJointLimits.limits[i].max.toDegrees(), theJointRequest.angles[i].toDegrees());
            OUTPUT_TEXT(tmp);
            clippedLastFrame[i] = theJointLimits.limits[i].max;
          }
        }
        else if(theJointRequest.angles[i] < theJointLimits.limits[i].min)
        {
          if(clippedLastFrame[i] != theJointLimits.limits[i].min)
          {
            char tmp[64];
            sprintf(tmp, "warning: clipped joint %s at %.04f, requested %.03f.", TypeRegistry::getEnumName(static_cast<Joints::Joint>(i)), theJointLimits.limits[i].min.toDegrees(), theJointRequest.angles[i].toDegrees());
            OUTPUT_TEXT(tmp);
            clippedLastFrame[i] = theJointLimits.limits[i].min;
          }
        }
        else
          clippedLastFrame[i] = SensorData::off;
      }
    }
  }

  float* actuators;
  naoBody.openActuators(actuators);
  int j = 0;
  ASSERT(headYawPositionActuator == 0);
  ASSERT(static_cast<int>(Joints::numOfJoints) - 1 == lbhNumOfPositionActuatorIds); //rHipYawPitch missin lbh

  for(int i = 0; i < Joints::numOfJoints; ++i, ++j)
  {
    if(i == Joints::rHipYawPitch) // missing on Nao
      ++i;

    if(theJointRequest.angles[i] == SensorData::off)
    {
      actuators[j] = 0.0f;
      actuators[j + lbhNumOfPositionActuatorIds] = 0.0f; // stiffness
    }
    else
    {
      actuators[j] = theJointRequest.angles[i] + theJointCalibration.offsets[i];
      actuators[j + lbhNumOfPositionActuatorIds] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[i]) / 100.f;
    }
  }
  j += lbhNumOfPositionActuatorIds;
  ASSERT(j == faceLedRedLeft0DegActuator);

  const LEDRequest& ledRequest(theLEDRequest);
  bool on = (theFrameInfo.time / 50 & 8) != 0;
  bool fastOn = (theFrameInfo.time / 10 & 8) != 0;
  for(int i = 0; i < LEDRequest::numOfLEDs; ++i)
    actuators[j++] = (ledRequest.ledStates[i] == LEDRequest::on ||
                      (ledRequest.ledStates[i] == LEDRequest::blinking && on) ||
                      (ledRequest.ledStates[i] == LEDRequest::fastBlinking && fastOn))
                     ? 1.0f : (ledRequest.ledStates[i] == LEDRequest::half ? 0.5f : 0.0f);

  naoBody.closeActuators();
  naoBody.setTeamInfo(Global::getSettings().teamNumber, Global::getSettings().teamColor, Global::getSettings().playerNumber);
}

void NaoProvider::update(FrameInfo& frameInfo)
{
  frameInfo.time = std::max(frameInfo.time + 1, Time::getCurrentSystemTime());

  if(gameControlData.packetNumber != naoBody.getGameControlData().packetNumber)
    gameControlTimeStamp = frameInfo.time;
  gameControlData = naoBody.getGameControlData();
}

void NaoProvider::update(FsrSensorData& fsrSensorData)
{
  float* sensors = naoBody.getSensors();

  FOREACH_ENUM(Legs::Leg, leg)
  {
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
      fsrSensorData.pressures[leg][sensor] = sensors[lFSRFrontLeftSensor + leg * FsrSensors::numOfFsrSensors + sensor];
    fsrSensorData.totals[leg] = sensors[lFSRTotalSensor + leg];
  }
}

void NaoProvider::update(InertialSensorData& inertialSensorData)
{
  float* sensors = naoBody.getSensors();

  // TODO: verify signs
  inertialSensorData.gyro.x() = sensors[gyroXSensor];
  inertialSensorData.gyro.y() = sensors[gyroYSensor];
  inertialSensorData.gyro.z() = -sensors[gyroZSensor]; // Aldebarans z-gyron is negated for some reason...

  inertialSensorData.acc.x() = -sensors[accXSensor];
  inertialSensorData.acc.y() = sensors[accYSensor];
  inertialSensorData.acc.z() = -sensors[accZSensor];

  inertialSensorData.angle.x() = sensors[angleXSensor];
  inertialSensorData.angle.y() = sensors[angleYSensor];
  inertialSensorData.angle.z() = -sensors[angleZSensor];
}

void NaoProvider::update(JointSensorData& jointSensorData)
{
  float* sensors = naoBody.getSensors();

  int j = 0;
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    if(i == Joints::rHipYawPitch)
    {
      jointSensorData.angles[i] = jointSensorData.angles[Joints::lHipYawPitch];
      jointSensorData.currents[i] = jointSensorData.currents[Joints::lHipYawPitch];
      jointSensorData.temperatures[i] = jointSensorData.temperatures[Joints::lHipYawPitch];
      jointSensorData.status[i] = jointSensorData.status[Joints::lHipYawPitch];
    }
    else
    {
      jointSensorData.angles[i] = sensors[j++] - theJointCalibration.offsets[i];
      jointSensorData.currents[i] = static_cast<short>(1000.f * sensors[j++]);
      jointSensorData.temperatures[i] = static_cast<unsigned char>(sensors[j++]);
      jointSensorData.status[i] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensors[j++]));
    }
  }
  jointSensorData.timestamp = theFrameInfo.time;
}

void NaoProvider::update(KeyStates& keyStates)
{
  float* sensors = naoBody.getSensors();

  for(int i = 0, j = headTouchFrontSensor; i < KeyStates::numOfKeys; ++i, ++j)
    keyStates.pressed[i] = sensors[j] != 0;
}

void NaoProvider::update(OpponentTeamInfo& opponentTeamInfo)
{
  (RoboCup::TeamInfo&) opponentTeamInfo = gameControlData.teams[gameControlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 1 : 0];
}

void NaoProvider::update(OwnTeamInfo& ownTeamInfo)
{
  (RoboCup::TeamInfo&) ownTeamInfo = gameControlData.teams[gameControlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
}

void NaoProvider::update(RawGameInfo& rawGameInfo)
{
  memcpy(&(RoboCup::RoboCupGameControlData&) rawGameInfo, &gameControlData, sizeof(gameControlData));
  rawGameInfo.timeLastPackageReceived = gameControlTimeStamp;
}

void NaoProvider::update(RobotInfo& robotInfo)
{
  RoboCup::TeamInfo& team = gameControlData.teams[gameControlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
  (RoboCup::RobotInfo&) robotInfo = team.players[Global::getSettings().playerNumber - 1];
  robotInfo.number = Global::getSettings().playerNumber;

  DEBUG_RESPONSE_ONCE("module:NaoProvider:robotInfo")
  {
    OUTPUT(idRobotInfo, bin, robotInfo);
  }
}

void NaoProvider::update(SystemSensorData& systemSensorData)
{
  float* sensors = naoBody.getSensors();
  if(theFrameInfo.getTimeSince(lastBodyTemperatureReadTime) * 1000 > 10)
  {
    lastBodyTemperatureReadTime = theFrameInfo.time;
    systemSensorData.cpuTemperature = naoBody.getCPUTemperature();
  }
  systemSensorData.batteryCurrent = sensors[batteryCurrentSensor];
  systemSensorData.batteryLevel = sensors[batteryChargeSensor];
  systemSensorData.batteryTemperature = sensors[batteryTemperatureSensor];
  const short statusValue = static_cast<short>(sensors[batteryStatusSensor]);
  systemSensorData.batteryCharging = statusValue & 0b10000000;
}

#endif // TARGET_ROBOT
