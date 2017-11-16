/**
 * @file Representations/Sensing/FootGroundContactStateProvider.cpp
 * @author Alexis Tsogias
 */

#include "FootGroundContactStateProvider.h"
#include "Tools/Math/Approx.h"

MAKE_MODULE(FootGroundContactStateProvider, sensing)

using Foot = FootGroundContactState::Foot;

FootGroundContactStateProvider::FootGroundContactStateProvider()
{
  leftZeroOffset.fill(initialMinPressure);
  rightZeroOffset.fill(initialMinPressure);
  leftMax.fill(initialMinPressure);
  rightMax.fill(initialMinPressure);
}

void FootGroundContactStateProvider::update(FootGroundContactState& footGroundContactState)
{
  for(unsigned i = 0; i < FsrSensors::numOfFsrSensors; ++i)
  {
    if(!theDamageConfigurationBody.sides[Legs::left].brokenFsrs[i] && theFsrSensorData.pressures[Legs::left][i] != SensorData::off)
    {
      leftZeroOffset[i] = std::min(leftZeroOffset[i], theFsrSensorData.pressures[Legs::left][i]);
      leftCalibrated[i] = theFsrSensorData.pressures[Legs::left][i] - leftZeroOffset[i];
      leftMax[i] = std::min(std::max(leftMax[i], leftCalibrated[i]), theMassCalibration.totalMass * 0.001f);
    }

    if(!theDamageConfigurationBody.sides[Legs::right].brokenFsrs[i] && theFsrSensorData.pressures[Legs::right][i] != SensorData::off)
    {
      rightZeroOffset[i] = std::min(rightZeroOffset[i], theFsrSensorData.pressures[Legs::right][i]);
      rightCalibrated[i] = theFsrSensorData.pressures[Legs::right][i] - rightZeroOffset[i];
      rightMax[i] = std::min(std::max(rightMax[i], rightCalibrated[i]), theMassCalibration.totalMass * 0.001f);
    }
  }
  float leftTotal = 0.f;
  float rightTotal = 0.f;
  Vector2f leftCoP = Vector2f::Zero();
  Vector2f rightCoP = Vector2f::Zero();
  bool contactLeft = false;
  bool contactRight = false;

  for(unsigned i = 0; i < FsrSensors::numOfFsrSensors; ++i)
  {
    if(!theDamageConfigurationBody.sides[Legs::left].brokenFsrs[i] && theFsrSensorData.pressures[Legs::left][i] != SensorData::off)
    {
      footGroundContactState.leftSensorContacts[i] = leftCalibrated[i] > leftMax[i] * noContactMargin;
      contactLeft |= footGroundContactState.leftSensorContacts[i];
      leftTotal += leftCalibrated[i];
      leftCoP += theRobotDimensions.leftFsrPositions[i] * leftCalibrated[i];
    }
    else
      footGroundContactState.leftSensorContacts[i] = false;

    if(!theDamageConfigurationBody.sides[Legs::right].brokenFsrs[i] && theFsrSensorData.pressures[Legs::right][i] != SensorData::off)
    {
      footGroundContactState.rightSensorContacts[i] = rightCalibrated[i] > rightMax[i] * noContactMargin;
      contactRight |= footGroundContactState.rightSensorContacts[i];
      rightTotal += rightCalibrated[i];
      rightCoP += theRobotDimensions.rightFsrPositions[i] * rightCalibrated[i];
    }
    else
      footGroundContactState.rightSensorContacts[i] = false;
  }
  const float bothTotal = leftTotal + rightTotal;

  if(contactLeft && contactRight)
    footGroundContactState.contact = FootGroundContactState::both;
  else if(contactLeft)
    footGroundContactState.contact = FootGroundContactState::left;
  else if(contactRight)
    footGroundContactState.contact = FootGroundContactState::right;
  else
    footGroundContactState.contact = FootGroundContactState::none;

  footGroundContactState.centerOfPressure[Legs::left] = leftCoP / leftTotal;
  footGroundContactState.centerOfPressure[Legs::right] = rightCoP / rightTotal;
  if(bothTotal != 0.f)
  {
    footGroundContactState.relativeMassDistribution[Legs::left] = leftTotal / bothTotal;
    footGroundContactState.relativeMassDistribution[Legs::right] = rightTotal / bothTotal;
  }
  else
    footGroundContactState.relativeMassDistribution.fill(0.f);

  footGroundContactState.totalMass[FootGroundContactState::left] = leftTotal;
  footGroundContactState.totalMass[FootGroundContactState::right] = rightTotal;
  footGroundContactState.totalMass[FootGroundContactState::both] = bothTotal;

  if((contactLeft || contactRight))
  {
    if(bothTotal > footGroundContactState.maxTotal)
      footGroundContactState.maxTotal = std::min(bothTotal, theMassCalibration.totalMass * 0.001f);
    else
      footGroundContactState.maxTotal *= (1 - maxLossRate * Constants::motionCycleTime);
  }
}
