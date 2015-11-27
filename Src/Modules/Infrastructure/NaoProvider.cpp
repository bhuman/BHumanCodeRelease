/**
 * @file Modules/Infrastructure/NaoProvider.cpp
 * The file declares a module that provides information from the Nao via DCM.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "NaoProvider.h"

MAKE_MODULE(NaoProvider, motionInfrastructure)

#ifdef TARGET_ROBOT

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Settings.h"

#include "libbhuman/bhuman.h"

#include <cstdio>
#include <cstring>
#include <algorithm>

PROCESS_LOCAL NaoProvider* NaoProvider::theInstance = nullptr;

NaoProvider::NaoProvider() :
  gameControlTimeStamp(0)
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
    if(Global::getSettings().robotName == Global::getSettings().bodyName)
      OUTPUT_TEXT("Hi, I am " << Global::getSettings().robotName << ".");
    else
      OUTPUT_TEXT("Hi, I am " << Global::getSettings().robotName << " (using " << Global::getSettings().bodyName << "s Body).");
    OUTPUT(idRobotname, bin, Global::getSettings().robotName << Global::getSettings().bodyName << Global::getSettings().location);
  }

  if(theInstance)
    theInstance->naoBody.wait();
}

void NaoProvider::send()
{
  DEBUG_RESPONSE("module:NaoProvider:lag100") SystemCall::sleep(100);
  DEBUG_RESPONSE("module:NaoProvider:lag200") SystemCall::sleep(200);
  DEBUG_RESPONSE("module:NaoProvider:lag300") SystemCall::sleep(300);
  DEBUG_RESPONSE("module:NaoProvider:lag1000") SystemCall::sleep(1000);
  DEBUG_RESPONSE("module:NaoProvider:lag3000") SystemCall::sleep(3000);
  DEBUG_RESPONSE("module:NaoProvider:lag6000") SystemCall::sleep(6000);
  DEBUG_RESPONSE("module:NaoProvider:segfault") *(volatile char*)0 = 0;

  DEBUG_RESPONSE("module:NaoProvider:ClippingInfo")
  {
    for(int i = 0; i < Joints::numOfJoints; ++i)
    {
      if(i == Joints::rHipYawPitch) // missing on Nao
        ++i;

      if(theJointRequest.angles[i] != SensorData::off)
      {
        if(theJointRequest.angles[i] > theJointCalibration.joints[i].maxAngle)
        {
          if(clippedLastFrame[i] != theJointCalibration.joints[i].maxAngle)
          {
            char tmp[64];
            sprintf(tmp, "warning: clipped joint %s at %.03f, requested %.03f.", Joints::getName(static_cast<Joints::Joint>(i)), theJointCalibration.joints[i].maxAngle.toDegrees(), theJointRequest.angles[i].toDegrees());
            OUTPUT_TEXT(tmp);
            clippedLastFrame[i] = theJointCalibration.joints[i].maxAngle;
          }
        }
        else if(theJointRequest.angles[i] < theJointCalibration.joints[i].minAngle)
        {
          if(clippedLastFrame[i] != theJointCalibration.joints[i].minAngle)
          {
            char tmp[64];
            sprintf(tmp, "warning: clipped joint %s at %.04f, requested %.03f.", Joints::getName(static_cast<Joints::Joint>(i)), theJointCalibration.joints[i].minAngle.toDegrees(), theJointRequest.angles[i].toDegrees());
            OUTPUT_TEXT(tmp);
            clippedLastFrame[i] = theJointCalibration.joints[i].minAngle;
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
      actuators[j] = theJointRequest.angles[i] + theJointCalibration.joints[i].offset;
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

  actuators[usActuator] = static_cast<float>(theUSRequest.sendMode);

  naoBody.closeActuators();
  naoBody.setTeamInfo(Global::getSettings().teamNumber, Global::getSettings().teamColor, Global::getSettings().playerNumber);
}

void NaoProvider::update(FrameInfo& frameInfo)
{
  frameInfo.time = std::max(frameInfo.time + 1, SystemCall::getCurrentSystemTime());
  frameInfo.cycleTime = 0.01f;

  if(gameControlData.packetNumber != naoBody.getGameControlData().packetNumber)
    gameControlTimeStamp = frameInfo.time;
  gameControlData = naoBody.getGameControlData();
}

void NaoProvider::update(FsrSensorData& fsrSensorData)
{
  float* sensors = naoBody.getSensors();

  for(size_t i = 0; i < fsrSensorData.left.size(); ++i)
    fsrSensorData.left[i] = sensors[lFSRFrontLeftSensor + i];
  for(size_t i = 0; i < fsrSensorData.right.size(); ++i)
    fsrSensorData.right[i] = sensors[rFSRFrontLeftSensor + i];
  fsrSensorData.leftTotal = sensors[lFSRTotalSensor];
  fsrSensorData.rightTotal = sensors[rFSRTotalSensor];
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
  
  PLOT("module:NaoProvider:gyroX", inertialSensorData.gyro.x().toDegrees());
  PLOT("module:NaoProvider:gyroY", inertialSensorData.gyro.y().toDegrees());
  PLOT("module:NaoProvider:gyroZ", inertialSensorData.gyro.z().toDegrees());
  PLOT("module:NaoProvider:accX", inertialSensorData.acc.x());
  PLOT("module:NaoProvider:accY", inertialSensorData.acc.y());
  PLOT("module:NaoProvider:accZ", inertialSensorData.acc.z());
  PLOT("module:NaoProvider:angleX", inertialSensorData.angle.x().toDegrees());
  PLOT("module:NaoProvider:angleY", inertialSensorData.angle.y().toDegrees());
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
    }
    else
    {
      jointSensorData.angles[i] = sensors[j++] - theJointCalibration.joints[i].offset;
      jointSensorData.currents[i] = static_cast<short>(1000.f * sensors[j++]);
      jointSensorData.temperatures[i] = static_cast<unsigned char>(sensors[j++]);
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
  memcpy(&(RoboCup::RoboCupGameControlData&) rawGameInfo, &gameControlData, (char*) gameControlData.teams - (char*) &gameControlData);
  rawGameInfo.timeLastPackageReceived = gameControlTimeStamp;
}

void NaoProvider::update(RobotInfo& robotInfo)
{
  RoboCup::TeamInfo& team = gameControlData.teams[gameControlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
  (RoboCup::RobotInfo&) robotInfo = team.players[Global::getSettings().playerNumber - 1];
  robotInfo.number = Global::getSettings().playerNumber;
  robotInfo.naoVersion = naoVersion;
  robotInfo.naoBodyType = naoBodyType;
  robotInfo.naoHeadType = naoHeadType;
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
}

void NaoProvider::update(UsSensorData& usSensorData)
{
  float* sensors = naoBody.getSensors();
  if(theUSRequest.receiveMode != -1)
  {
    ASSERT(usSensorData.left.size() == lUs9Sensor - lUsSensor + 1);
    for(size_t i = 0; i < usSensorData.left.size(); ++i)
    {
      float data = sensors[lUsSensor + i];
      usSensorData.left[i] = data != 0.f ? data * 1000.f : SensorData::off;
    }

    ASSERT(usSensorData.right.size() == rUs9Sensor - rUsSensor + 1);
    for(size_t i = 0; i < usSensorData.right.size(); ++i)
    {
      float data = sensors[rUsSensor + i];
      usSensorData.right[i] = data != 0.f ? data * 1000.f : SensorData::off;
    }

    usSensorData.actuatorMode = static_cast<UsSensorData::UsActuatorMode>(theUSRequest.receiveMode);
    usSensorData.timeStamp = theFrameInfo.time;
  }

  PLOT("module:NaoProvider:usLeft", usSensorData.left[0]);
  PLOT("module:NaoProvider:usRight", usSensorData.right[0]);
}

#endif // TARGET_ROBOT
