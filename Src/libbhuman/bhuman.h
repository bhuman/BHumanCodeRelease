/**
 * @file bhuman.h
 * Declaration of shared data provided by libbhuman.
 */

#pragma once

#include "Representations/Infrastructure/RoboCupGameControlData.h"

#define LBH_SEM_NAME "/bhuman_sem"
#define LBH_MEM_NAME "/bhuman_mem"

enum NAOVersion
{
  V32,
  V33,
  V4,
  V5,
  V6,
};

enum NAOType
{
  H21,
  H25,
};

enum LBHSensorIds
{
  // joint sensors
  headYawPositionSensor,
  headYawCurrentSensor,
  headYawTemperatureSensor,
  headYawTemperatureStatusSensor,
  headPitchPositionSensor,
  headPitchCurrentSensor,
  headPitchTemperatureSensor,
  headPitchTemperatureStatusSensor,
  lShoulderPitchPositionSensor,
  lShoulderPitchCurrentSensor,
  lShoulderPitchTemperatureSensor,
  lShoulderPitchTemperatureStatusSensor,
  lShoulderRollPositionSensor,
  lShoulderRollCurrentSensor,
  lShoulderRollTemperatureSensor,
  lShoulderRollTemperatureStatusSensor,
  lElbowYawPositionSensor,
  lElbowYawCurrentSensor,
  lElbowYawTemperatureSensor,
  lElbowYawTemperatureStatusSensor,
  lElbowRollPositionSensor,
  lElbowRollCurrentSensor,
  lElbowRollTemperatureSensor,
  lElbowRollTemperatureStatusSensor,
  lWristYawPositionSensor,
  lWristYawCurrentSensor,
  lWristYawTemperatureSensor,
  lWristYawTemperatureStatusSensor,
  lHandPositionSensor,
  lHandCurrentSensor,
  lHandTemperatureSensor,
  lHandTemperatureStatusSensor,
  rShoulderPitchPositionSensor,
  rShoulderPitchCurrentSensor,
  rShoulderPitchTemperatureSensor,
  rShoulderPitchTemperatureStatusSensor,
  rShoulderRollPositionSensor,
  rShoulderRollCurrentSensor,
  rShoulderRollTemperatureSensor,
  rShoulderRollTemperatureStatusSensor,
  rElbowYawPositionSensor,
  rElbowYawCurrentSensor,
  rElbowYawTemperatureSensor,
  rElbowYawTemperatureStatusSensor,
  rElbowRollPositionSensor,
  rElbowRollCurrentSensor,
  rElbowRollTemperatureSensor,
  rElbowRollTemperatureStatusSensor,
  rWristYawPositionSensor,
  rWristYawCurrentSensor,
  rWristYawTemperatureSensor,
  rWristYawTemperatureStatusSensor,
  rHandPositionSensor,
  rHandCurrentSensor,
  rHandTemperatureSensor,
  rHandTemperatureStatusSensor,
  lHipYawPitchPositionSensor,
  lHipYawPitchCurrentSensor,
  lHipYawPitchTemperatureSensor,
  lHipYawPitchTemperatureStatusSensor,
  lHipRollPositionSensor,
  lHipRollCurrentSensor,
  lHipRollTemperatureSensor,
  lHipRollTemperatureStatusSensor,
  lHipPitchPositionSensor,
  lHipPitchCurrentSensor,
  lHipPitchTemperatureSensor,
  lHipPitchTemperatureStatusSensor,
  lKneePitchPositionSensor,
  lKneePitchCurrentSensor,
  lKneePitchTemperatureSensor,
  lKneePitchTemperatureStatusSensor,
  lAnklePitchPositionSensor,
  lAnklePitchCurrentSensor,
  lAnklePitchTemperatureSensor,
  lAnklePitchTemperatureStatusSensor,
  lAnkleRollPositionSensor,
  lAnkleRollCurrentSensor,
  lAnkleRollTemperatureSensor,
  lAnkleRollTemperatureStatusSensor,
  rHipRollPositionSensor,
  rHipRollCurrentSensor,
  rHipRollTemperatureSensor,
  rHipRollTemperatureStatusSensor,
  rHipPitchPositionSensor,
  rHipPitchCurrentSensor,
  rHipPitchTemperatureSensor,
  rHipPitchTemperatureStatusSensor,
  rKneePitchPositionSensor,
  rKneePitchCurrentSensor,
  rKneePitchTemperatureSensor,
  rKneePitchTemperatureStatusSensor,
  rAnklePitchPositionSensor,
  rAnklePitchCurrentSensor,
  rAnklePitchTemperatureSensor,
  rAnklePitchTemperatureStatusSensor,
  rAnkleRollPositionSensor,
  rAnkleRollCurrentSensor,
  rAnkleRollTemperatureSensor,
  rAnkleRollTemperatureStatusSensor,

  // touch sensors
  headTouchFrontSensor,
  headTouchMiddleSensor,
  headTouchRearSensor,
  lHandTouchBackSensor,
  lHandTouchLeftSensor,
  lHandTouchRightSensor,
  rHandTouchBackSensor,
  rHandTouchLeftSensor,
  rHandTouchRightSensor,

  // switches
  lBumperLeftSensor,
  lBumperRightSensor,
  rBumperLeftSensor,
  rBumperRightSensor,
  chestButtonSensor,

  // inertial sensors
  gyroXSensor,
  gyroYSensor,
  gyroZSensor,
  accXSensor,
  accYSensor,
  accZSensor,
  angleXSensor,
  angleYSensor,
  angleZSensor,

  // battery sensors
  batteryCurrentSensor,
  batteryChargeSensor,
  batteryStatusSensor,
  batteryTemperatureSensor,
  batteryTemperatureStatusSensor,

  // fsr sensors
  lFSRFrontLeftSensor,
  lFSRFrontRightSensor,
  lFSRRearLeftSensor,
  lFSRRearRightSensor,
  rFSRFrontLeftSensor,
  rFSRFrontRightSensor,
  rFSRRearLeftSensor,
  rFSRRearRightSensor,
  lFSRTotalSensor,
  rFSRTotalSensor,

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
  lWristYawPositionActuator,
  lHandPositionActuator,
  rShoulderPitchPositionActuator,
  rShoulderRollPositionActuator,
  rElbowYawPositionActuator,
  rElbowRollPositionActuator,
  rWristYawPositionActuator,
  rHandPositionActuator,
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
  lbhNumOfPositionActuatorIds,

  // stiffness request
  headYawStiffnessActuator = lbhNumOfPositionActuatorIds,
  headPitchStiffnessActuator,
  lShoulderPitchStiffnessActuator,
  lShoulderRollStiffnessActuator,
  lElbowYawStiffnessActuator,
  lElbowRollStiffnessActuator,
  lWristYawStiffnessActuator,
  lHandStiffnessActuator,
  rShoulderPitchStiffnessActuator,
  rShoulderRollStiffnessActuator,
  rElbowYawStiffnessActuator,
  rElbowRollStiffnessActuator,
  rWristYawStiffnessActuator,
  rHandStiffnessActuator,
  lHipYawPitchStiffnessActuator,
  lHipRollStiffnessActuator,
  lHipPitchStiffnessActuator,
  lKneePitchStiffnessActuator,
  lAnklePitchStiffnessActuator,
  lAnkleRollStiffnessActuator,
  rHipRollStiffnessActuator,
  rHipPitchStiffnessActuator,
  rKneePitchStiffnessActuator,
  rAnklePitchStiffnessActuator,
  rAnkleRollStiffnessActuator,

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
  earsLedLeft0DegActuator,
  earsLedLeft36DegActuator,
  earsLedLeft72DegActuator,
  earsLedLeft108DegActuator,
  earsLedLeft144DegActuator,
  earsLedLeft180DegActuator,
  earsLedLeft216DegActuator,
  earsLedLeft252DegActuator,
  earsLedLeft288DegActuator,
  earsLedLeft324DegActuator,
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
  headLedRearLeft0Actuator,
  headLedRearLeft1Actuator,
  headLedRearLeft2Actuator,
  headLedRearRight0Actuator,
  headLedRearRight1Actuator,
  headLedRearRight2Actuator,
  headLedMiddleRight0Actuator,
  headLedFrontRight0Actuator,
  headLedFrontRight1Actuator,
  headLedFrontLeft0Actuator,
  headLedFrontLeft1Actuator,
  headLedMiddleLeft0Actuator,
  lFootLedRedActuator,
  lFootLedGreenActuator,
  lFootLedBlueActuator,
  rFootLedRedActuator,
  rFootLedGreenActuator,
  rFootLedBlueActuator,

  lbhNumOfActuatorIds
};

enum LBHTeamInfoIds
{
  teamNumber,
  teamColor,
  playerNumber,
  lbhNumOfTeamInfoIds
};

const int lbhNumOfStiffnessActuatorIds = lbhNumOfPositionActuatorIds;
const int lbhNumOfLedActuatorIds = rFootLedBlueActuator + 1 - faceLedRedLeft0DegActuator;
const int lbhNumOfDifSensors = 4;

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

  char bodyId[21]; /* Device/DeviceList/ChestBoard/BodyId */
  char headId[21]; /* RobotConfig/Head/FullHeadId */
  NAOVersion headVersion;
  NAOVersion bodyVersion;
  NAOType headType;
  NAOType bodyType;
  float sensors[3][lbhNumOfSensorIds];
  float actuators[3][lbhNumOfActuatorIds];
  RoboCup::RoboCupGameControlData gameControlData[3];

  BHState state;
  int teamInfo[lbhNumOfTeamInfoIds];
  unsigned bhumanStartTime;
};
