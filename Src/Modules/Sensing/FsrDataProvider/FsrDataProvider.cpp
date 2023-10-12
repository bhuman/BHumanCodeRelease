/**
 * @file FsrDataProvider.h
 * This modules calibrates the fsr sensor data
 * @author Philip Reichenberg
 */

#include "FsrDataProvider.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(FsrDataProvider);

FsrDataProvider::FsrDataProvider()
{
  FOREACH_ENUM(Legs::Leg, leg)
  {
    highestLegSumPressure[leg] = minPressure;
    newHighestLegSumPressure[leg] = minPressure;
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
    {
      highestPressure[leg][sensor] = minPressure;
      newHighestPressure[leg][sensor] = minPressure;
      newLowestPressure[leg][sensor] = maxPressure;
      lowestPressure[leg][sensor] = 0.f;
    }
  }

  maxFootPressureCurrent = minPressure;
}

void FsrDataProvider::update(FsrData& theFsrData)
{
  if((isCalibrated || SystemCall::getMode() == SystemCall::simulatedRobot) && !theFsrData.isCalibrated)
  {
    theFsrData.minPressure = minPressurePercent;
    theFsrData.isCalibrated = true;
    isCalibrated = true;
  }
  else if(!theFsrData.isCalibrated)
    theFsrData.minPressure = minPressurePercent;

  // Go through every fsr sensor and get:
  // - the sum pressure per leg
  // - % pressure per sensor, based on its max measured pressure
  float leftLegPressure = 0.f;
  float rightLegPressure = 0.f;
  FOREACH_ENUM(Legs::Leg, leg)
  {
    float backwardPressure = 0.f;
    float forwardPressure = 0.f;
    float leftPressure = 0.f;
    float rightPressure = 0.f;
    float& footPressure = leg == Legs::left ? leftLegPressure : rightLegPressure;
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
    {
      // measured weight - offset, clipped to maxPressure
      float pressure = std::max(0.f, std::min(maxPressure, theFsrSensorData.pressures[leg][sensor] != SensorData::off ? theFsrSensorData.pressures[leg][sensor] : 0.f) - lowestPressure[leg][sensor]);
      footPressure += pressure;
      // Safe all time min and max weights per sensor
      highestPressure[leg][sensor] = std::max(highestPressure[leg][sensor], pressure);
      newHighestPressure[leg][sensor] = std::max(newHighestPressure[leg][sensor], pressure);
      newLowestPressure[leg][sensor] = std::min(newLowestPressure[leg][sensor], pressure);
      // % max per sensor
      theFsrData.pressures[leg][sensor] = pressure / highestPressure[leg][sensor];

      if(sensor == FsrSensors::bl || sensor == FsrSensors::br)
        backwardPressure += pressure;
      else
        forwardPressure += pressure;
      if(sensor == FsrSensors::bl || sensor == FsrSensors::fl)
        leftPressure += pressure;
      else
        rightPressure += pressure;
    }
    // safe all time max per leg
    maxFootPressureCurrent = std::max(maxFootPressureCurrent, footPressure);
    highestLegSumPressure[leg] = std::max(highestLegSumPressure[leg], footPressure);
    newHighestLegSumPressure[leg] = std::max(newHighestLegSumPressure[leg], footPressure);
    // % max per leg
    theFsrData.legInfo[leg].totals = footPressure / highestLegSumPressure[leg];

    // Update last pressure time stamps
    const unsigned lastHasPressure = theFsrData.legInfo[leg].hasPressure;
    theFsrData.legInfo[leg].hasPressure = footPressure > minPressure ? theFrameInfo.time : theFsrData.legInfo[leg].hasPressure;
    theFsrData.legInfo[leg].backwardPressure = backwardPressure > minPressure * minSingleFSRPressureForPredictedSwitchFactor ? theFrameInfo.time : theFsrData.legInfo[leg].backwardPressure;
    theFsrData.legInfo[leg].forwardPressure = forwardPressure > minPressure * minSingleFSRPressureForPredictedSwitchFactor ? theFrameInfo.time : theFsrData.legInfo[leg].forwardPressure;
    theFsrData.legInfo[leg].leftPressure = leftPressure > minPressure * minSingleFSRPressureForPredictedSwitchFactor ? theFrameInfo.time : theFsrData.legInfo[leg].leftPressure;
    theFsrData.legInfo[leg].rightPressure = rightPressure > minPressure * minSingleFSRPressureForPredictedSwitchFactor ? theFrameInfo.time : theFsrData.legInfo[leg].rightPressure;
    theFsrData.legInfo[leg].hasPressureSince = theFsrData.legInfo[leg].hasPressure == theFrameInfo.time && lastHasPressure != theFsrData.lastUpdateTimestamp ? theFsrData.legInfo[leg].hasPressure : theFsrData.legInfo[leg].hasPressureSince;
    theFsrData.legInfo[leg].sagittalRatio = backwardPressure + forwardPressure > 0.f ? forwardPressure / (backwardPressure + forwardPressure) : 0.f;
    theFsrData.legInfo[leg].lateralRatio = leftPressure + rightPressure > 0.f ? leftPressure / (leftPressure + rightPressure) : 0.f;
  }

  if(!theMotionInfo.isMotion(MotionPhase::walk)
     || theFrameInfo.getTimeSince(theFsrData.legInfo[Legs::left].hasPressure) > maxTimeLegNoPressureForCalibration
     || theFrameInfo.getTimeSince(theFsrData.legInfo[Legs::right].hasPressure) > maxTimeLegNoPressureForCalibration)
    updatePressureTimestamp = theFrameInfo.time;

  // Update the heighest pressures
  if(theFrameInfo.getTimeSince(updatePressureTimestamp) > highestPressureUpdateTime && theFootSupport.switched)
  {
    updatePressureTimestamp = theFrameInfo.time;
    FOREACH_ENUM(Legs::Leg, leg)
    {
      highestLegSumPressure[leg] = newHighestLegSumPressure[leg];
      newHighestLegSumPressure[leg] = minPressure;
      FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
      {
        highestPressure[leg][sensor] = newHighestPressure[leg][sensor];
        newHighestPressure[leg][sensor] = minPressure;
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
  if(supportSwitchCounter >= numOfSupportSwitches)
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
    minPressure = (minPressureRange.max - minPressureRange.min) * factor + minPressureRange.min;
    minPressurePercent = minPressure / maxFootSumPressure;
    theFsrData.minPressure = minPressurePercent;
    supportSwitchCounter = 0;
    maxFootSumPressure = 0;
    theFsrData.isCalibrated = true;
  }

  theFsrData.lastUpdateTimestamp = theFrameInfo.time;
}
