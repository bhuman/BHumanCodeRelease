/**
* @file bhuman.h
* Declaration of shared data provided by libbhuman.
*/

#pragma once

#include "Representations/Infrastructure/RoboCupGameControlData.h"

#define LBH_SEM_NAME "/bhuman_sem"
#define LBH_MEM_NAME "/bhuman_mem"

enum LBHSensorIds
{
  // joint data
  headYawPositionSensor,
  headYawCurrentSensor,
  headYawTemperatureSensor,
  headPitchPositionSensor,
  headPitchCurrentSensor,
  headPitchTemperatureSensor,
  lShoulderPitchPositionSensor,
  lShoulderPitchCurrentSensor,
  lShoulderPitchTemperatureSensor,
  lShoulderRollPositionSensor,
  lShoulderRollCurrentSensor,
  lShoulderRollTemperatureSensor,
  lElbowYawPositionSensor,
  lElbowYawCurrentSensor,
  lElbowYawTemperatureSensor,
  lElbowRollPositionSensor,
  lElbowRollCurrentSensor,
  lElbowRollTemperatureSensor,
  rShoulderPitchPositionSensor,
  rShoulderPitchCurrentSensor,
  rShoulderPitchTemperatureSensor,
  rShoulderRollPositionSensor,
  rShoulderRollCurrentSensor,
  rShoulderRollTemperatureSensor,
  rElbowYawPositionSensor,
  rElbowYawCurrentSensor,
  rElbowYawTemperatureSensor,
  rElbowRollPositionSensor,
  rElbowRollCurrentSensor,
  rElbowRollTemperatureSensor,
  lHipYawPitchPositionSensor,
  lHipYawPitchCurrentSensor,
  lHipYawPitchTemperatureSensor,
  lHipRollPositionSensor,
  lHipRollCurrentSensor,
  lHipRollTemperatureSensor,
  lHipPitchPositionSensor,
  lHipPitchCurrentSensor,
  lHipPitchTemperatureSensor,
  lKneePitchPositionSensor,
  lKneePitchCurrentSensor,
  lKneePitchTemperatureSensor,
  lAnklePitchPositionSensor,
  lAnklePitchCurrentSensor,
  lAnklePitchTemperatureSensor,
  lAnkleRollPositionSensor,
  lAnkleRollCurrentSensor,
  lAnkleRollTemperatureSensor,
  rHipRollPositionSensor,
  rHipRollCurrentSensor,
  rHipRollTemperatureSensor,
  rHipPitchPositionSensor,
  rHipPitchCurrentSensor,
  rHipPitchTemperatureSensor,
  rKneePitchPositionSensor,
  rKneePitchCurrentSensor,
  rKneePitchTemperatureSensor,
  rAnklePitchPositionSensor,
  rAnklePitchCurrentSensor,
  rAnklePitchTemperatureSensor,
  rAnkleRollPositionSensor,
  rAnkleRollCurrentSensor,
  rAnkleRollTemperatureSensor,

  // sensor data
  gyroXSensor,
  gyroYSensor,
  gyroRefSensor,
  accXSensor,
  accYSensor,
  accZSensor,
  batteryChargeSensor,
  lFSRFrontLeftSensor,
  lFSRFrontRightSensor,
  lFSRRearLeftSensor,
  lFSRRearRightSensor,
  rFSRFrontLeftSensor,
  rFSRFrontRightSensor,
  rFSRRearLeftSensor,
  rFSRRearRightSensor,
  lUsSensor,
  lUs1Sensor,
  lUs2Sensor,
  lUs3Sensor,
  lUs4Sensor,
  lUs5Sensor,
  lUs6Sensor,
  lUs7Sensor,
  lUs8Sensor,
  lUs9Sensor,
  rUsSensor,
  rUs1Sensor,
  rUs2Sensor,
  rUs3Sensor,
  rUs4Sensor,
  rUs5Sensor,
  rUs6Sensor,
  rUs7Sensor,
  rUs8Sensor,
  rUs9Sensor,
  angleXSensor,
  angleYSensor,

  // key states
  rBumperRightSensor,
  rBumperLeftSensor,
  lBumperRightSensor,
  lBumperLeftSensor,
  chestButtonSensor,

  lbhNumOfSensorIds,
};

enum LBHActuatorIds
{
  // joint request
  headYawPositionActuator,
  headPitchPositionActuator,
  lShoulderPitchPositionActuator,
  lShoulderRollPositionActuator,
  lElbowYawPositionActuator,
  lElbowRollPositionActuator,
  rShoulderPitchPositionActuator,
  rShoulderRollPositionActuator,
  rElbowYawPositionActuator,
  rElbowRollPositionActuator,
  lHipYawPitchPositionActuator,
  lHipRollPositionActuator,
  lHipPitchPositionActuator,
  lKneePitchPositionActuator,
  lAnklePitchPositionActuator,
  lAnkleRollPositionActuator,
  rHipRollPositionActuator,
  rHipPitchPositionActuator,
  rKneePitchPositionActuator,
  rAnklePitchPositionActuator,
  rAnkleRollPositionActuator,
  lHandPositionActuator,
  rHandPositionActuator,
  lbhNumOfPositionActuatorIds,

  headYawHardnessActuator = lbhNumOfPositionActuatorIds,
  headPitchHardnessActuator,
  lShoulderPitchHardnessActuator,
  lShoulderRollHardnessActuator,
  lElbowYawHardnessActuator,
  lElbowRollHardnessActuator,
  rShoulderPitchHardnessActuator,
  rShoulderRollHardnessActuator,
  rElbowYawHardnessActuator,
  rElbowRollHardnessActuator,
  lHipYawPitchHardnessActuator,
  lHipRollHardnessActuator,
  lHipPitchHardnessActuator,
  lKneePitchHardnessActuator,
  lAnklePitchHardnessActuator,
  lAnkleRollHardnessActuator,
  rHipRollHardnessActuator,
  rHipPitchHardnessActuator,
  rKneePitchHardnessActuator,
  rAnklePitchHardnessActuator,
  rAnkleRollHardnessActuator,
  lHandHardnessActuator,
  rHandHardnessActuator,

  // led request
  faceLedRedLeft0DegActuator,
  faceLedRedLeft45DegActuator,
  faceLedRedLeft90DegActuator,
  faceLedRedLeft135DegActuator,
  faceLedRedLeft180DegActuator,
  faceLedRedLeft225DegActuator,
  faceLedRedLeft270DegActuator,
  faceLedRedLeft315DegActuator,
  faceLedGreenLeft0DegActuator,
  faceLedGreenLeft45DegActuator,
  faceLedGreenLeft90DegActuator,
  faceLedGreenLeft135DegActuator,
  faceLedGreenLeft180DegActuator,
  faceLedGreenLeft225DegActuator,
  faceLedGreenLeft270DegActuator,
  faceLedGreenLeft315DegActuator,
  faceLedBlueLeft0DegActuator,
  faceLedBlueLeft45DegActuator,
  faceLedBlueLeft90DegActuator,
  faceLedBlueLeft135DegActuator,
  faceLedBlueLeft180DegActuator,
  faceLedBlueLeft225DegActuator,
  faceLedBlueLeft270DegActuator,
  faceLedBlueLeft315DegActuator,
  faceLedRedRight0DegActuator,
  faceLedRedRight45DegActuator,
  faceLedRedRight90DegActuator,
  faceLedRedRight135DegActuator,
  faceLedRedRight180DegActuator,
  faceLedRedRight225DegActuator,
  faceLedRedRight270DegActuator,
  faceLedRedRight315DegActuator,
  faceLedGreenRight0DegActuator,
  faceLedGreenRight45DegActuator,
  faceLedGreenRight90DegActuator,
  faceLedGreenRight135DegActuator,
  faceLedGreenRight180DegActuator,
  faceLedGreenRight225DegActuator,
  faceLedGreenRight270DegActuator,
  faceLedGreenRight315DegActuator,
  faceLedBlueRight0DegActuator,
  faceLedBlueRight45DegActuator,
  faceLedBlueRight90DegActuator,
  faceLedBlueRight135DegActuator,
  faceLedBlueRight180DegActuator,
  faceLedBlueRight225DegActuator,
  faceLedBlueRight270DegActuator,
  faceLedBlueRight315DegActuator,
  earsLedLeft36DegActuator,
  earsLedLeft72DegActuator,
  earsLedLeft108DegActuator,
  earsLedLeft144DegActuator,
  earsLedLeft180DegActuator,
  earsLedLeft216DegActuator,
  earsLedLeft252DegActuator,
  earsLedLeft288DegActuator,
  earsLedLeft324DegActuator,
  earsLedLeft0DegActuator,
  earsLedRight0DegActuator,
  earsLedRight36DegActuator,
  earsLedRight72DegActuator,
  earsLedRight108DegActuator,
  earsLedRight144DegActuator,
  earsLedRight180DegActuator,
  earsLedRight216DegActuator,
  earsLedRight252DegActuator,
  earsLedRight288DegActuator,
  earsLedRight324DegActuator,
  chestBoardLedRedActuator,
  chestBoardLedGreenActuator,
  chestBoardLedBlueActuator,
  lFootLedRedActuator,
  lFootLedGreenActuator,
  lFootLedBlueActuator,
  rFootLedRedActuator,
  rFootLedGreenActuator,
  rFootLedBlueActuator,
  lbhNumOfNormalActuatorIds,

  usActuator = lbhNumOfNormalActuatorIds,

  lbhNumOfActuatorIds
};

enum LBHTeamInfoIds
{
  teamNumber,
  teamColour,
  playerNumber,
  lbhNumOfTeamInfoIds
};

const int lbhNumOfHardnessActuatorIds = rHandHardnessActuator + 1 - headYawHardnessActuator;
const int lbhNumOfLedActuatorIds = rFootLedBlueActuator + 1 - faceLedRedLeft0DegActuator;

enum BHState
{
  okState = 0,
  abnormalTerminationState = 1,
  sigINTState = 2,
  sigQUITState = 3,
  sigILLState = 4,
  sigABRTState = 6,
  sigFPEState = 8,
  sigKILLState = 9,
  sigSEGVState = 11,
  sigPIPEState = 13,
  sigALRMState = 14,
  sigTERMState = 15,
};

struct LBHData
{
  volatile int readingSensors; /**< Index of sensor data reserved for reading. */
  volatile int newestSensors; /**< Index of the newest sensor data. */
  volatile int readingActuators; /**< Index of actuator commands reserved for reading. */
  volatile int newestActuators; /**< Index of the newest actuator command. */

  char robotName[24]; /* Device/DeviceList/ChestBoard/BodyNickName */
  float sensors[3][lbhNumOfSensorIds];
  float actuators[3][lbhNumOfActuatorIds];
  RoboCup::RoboCupGameControlData gameControlData[3];

  BHState state;
  int teamInfo[lbhNumOfTeamInfoIds];
  unsigned bhumanStartTime;
};
