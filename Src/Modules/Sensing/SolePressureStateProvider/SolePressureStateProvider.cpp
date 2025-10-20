/**
 * @file SolePressureStateProvider.h
 * This modules calibrates the fsr sensor data
 * @author Philip Reichenberg
 */

#include "SolePressureStateProvider.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(SolePressureStateProvider);

SolePressureStateProvider::SolePressureStateProvider()
{
  FOREACH_ENUM(Legs::Leg, leg)
  {
    highestLegSumPressure[leg] = minPressure.max;
    newHighestLegSumPressure[leg] = minPressure.max;
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
    {
      highestPressure[leg][sensor] = minPressure.max;
      newHighestPressure[leg][sensor] = minPressure.max;
      newLowestPressure[leg][sensor] = maxPressure;
    }
    originalHasPressure[leg] = false;
  }

  maxFootPressureCurrent = minPressure.max;
}

void SolePressureStateProvider::update(SolePressureState& theSolePressureState)
{
  if((isCalibrated || SystemCall::getMode() == SystemCall::simulatedRobot) && !theSolePressureState.isCalibrated)
  {
    theSolePressureState.minPressure = minPressurePercent;
    theSolePressureState.isCalibrated = true;
    isCalibrated = true;
  }
  else if(!theSolePressureState.isCalibrated)
    theSolePressureState.minPressure = minPressurePercent;

  // Go through every fsr sensor and get:
  // - the sum pressure per leg
  // - % pressure per sensor, based on its max measured pressure
  float leftLegPressure = 0.f;
  float rightLegPressure = 0.f;
  FOREACH_ENUM(Legs::Leg, leg)
  {
    float& footPressure = leg == Legs::left ? leftLegPressure : rightLegPressure;
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
    {
      if(std::find(theDamageConfigurationBody.brokenFsrSensors[leg].begin(), theDamageConfigurationBody.brokenFsrSensors[leg].end(), sensor) != theDamageConfigurationBody.brokenFsrSensors[leg].end())
      {
        theSolePressureState.pressures[leg][sensor] = 0.f;
        continue;
      }
      // measured weight - offset, clipped to maxPressure
      const float rawPressure = std::min(maxPressure, theFsrSensorData.pressures[leg][sensor] != SensorData::off ? theFsrSensorData.pressures[leg][sensor] : 0.f);
      float pressure = std::max(0.f, rawPressure - lowestPressure[leg][sensor]);
      footPressure += pressure;
      // Safe all time min and max weights per sensor
      highestPressure[leg][sensor] = std::max(highestPressure[leg][sensor], pressure);
      newHighestPressure[leg][sensor] = std::max(newHighestPressure[leg][sensor], pressure);
      newLowestPressure[leg][sensor] = std::min(newLowestPressure[leg][sensor], rawPressure);
      // % max per sensor
      theSolePressureState.pressures[leg][sensor] = pressure / highestPressure[leg][sensor];
    }
    // safe all time max per leg
    maxFootPressureCurrent = std::max(maxFootPressureCurrent, footPressure);
    highestLegSumPressure[leg] = std::max(highestLegSumPressure[leg], footPressure);
    newHighestLegSumPressure[leg] = std::max(newHighestLegSumPressure[leg], footPressure);
    // % max per leg
    theSolePressureState.legInfo[leg].totals = footPressure / highestLegSumPressure[leg];

    // Update last pressure time stamps
    theSolePressureState.legInfo[leg].hasPressure = footPressure > minPressure.max;
    originalHasPressure[leg] = theSolePressureState.legInfo[leg].hasPressure;
    theSolePressureState.legInfo[leg].hasPressureSince = theSolePressureState.legInfo[leg].hasPressure ? theFrameInfo.time : theSolePressureState.legInfo[leg].hasPressureSince;

    // Handling in case FSRs might need a new calibration
    if(!theSolePressureState.legInfo[leg].hasPressure)
      theSolePressureState.legInfo[leg].hasPressure = footPressure > mapToRange(static_cast<float>(theFrameInfo.getTimeSince(theSolePressureState.legInfo[leg].hasPressureSince)), badMinPressureTimewindow.min, badMinPressureTimewindow.max, minPressure.max, minPressure.min);
  }

  if(!theMotionInfo.isMotion(MotionPhase::walk)
     || theFrameInfo.getTimeSince(theSolePressureState.legInfo[Legs::left].hasPressure) > maxTimeLegNoPressureForCalibration
     || theFrameInfo.getTimeSince(theSolePressureState.legInfo[Legs::right].hasPressure) > maxTimeLegNoPressureForCalibration)
    updatePressureTimestamp = theFrameInfo.time;

  // Update the highest pressures
  if(theFrameInfo.getTimeSince(updatePressureTimestamp) > highestPressureUpdateTime && theFootSupport.switched)
  {
    updatePressureTimestamp = theFrameInfo.time;
    FOREACH_ENUM(Legs::Leg, leg)
    {
      highestLegSumPressure[leg] = newHighestLegSumPressure[leg];
      newHighestLegSumPressure[leg] = minPressure.max;
      FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
      {
        highestPressure[leg][sensor] = newHighestPressure[leg][sensor];
        newHighestPressure[leg][sensor] = minPressure.max;
      }
    }
  }

  // Increase supportSwitchCounter
  if(theFootSupport.switched
     && theFrameInfo.getTimeSince(lastSupportSwitch) < maxTimeBetweenSupportSwitches && theFrameInfo.getTimeSince(lastSupportSwitch) > minTimeBetweenSupportSwitches
     && theMotionInfo.executedPhase == MotionPhase::walk)
  {
    ++supportSwitchCounter;
    lastSupportSwitch = theFrameInfo.time;
    maxFootSumPressure += maxFootPressureCurrent;
    maxFootPressureCurrent = 0.f;
  }
  else if(theFootSupport.switched)
    lastSupportSwitch = theFrameInfo.time;
  if(theMotionInfo.executedPhase != MotionPhase::walk)
    supportSwitchCounter = 0;

  // Enough support switches happened. Update the lowest measured pressures and calibrate the minPressure
  if(allowRecalibration && supportSwitchCounter >= numOfSupportSwitches)
  {
    // Calibrate lowestPressure offsets
    FOREACH_ENUM(Legs::Leg, leg)
      FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
      {
        lowestPressure[leg][sensor] = newLowestPressure[leg][sensor];
        newLowestPressure[leg][sensor] = maxPressure;
        if(lowestPressure[leg][sensor] > 0.01f && SystemCall::getMode() != SystemCall::simulatedRobot)
          OUTPUT_TEXT("fsr" << TypeRegistry::getEnumName(leg) << " " << TypeRegistry::getEnumName(sensor) << " has offsets of " << lowestPressure[leg][sensor]);
      }

    maxFootSumPressure /= supportSwitchCounter;
    const float factor = Rangef::ZeroOneRange().limit((maxFootSumPressure - minPressureInterpolationValues.min) / (minPressureInterpolationValues.max - minPressureInterpolationValues.min));
    minPressure.max = (minPressureRange.max - minPressureRange.min) * factor + minPressureRange.min;
    minPressurePercent = minPressure.max / maxFootSumPressure;
    theSolePressureState.minPressure = minPressurePercent;
    supportSwitchCounter = 0;
    maxFootSumPressure = 0;
    theSolePressureState.isCalibrated = true;
  }
}
