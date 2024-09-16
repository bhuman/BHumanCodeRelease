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
    highestLegSumPressure[leg] = minPressure.max;
    newHighestLegSumPressure[leg] = minPressure.max;
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
    {
      highestPressure[leg][sensor] = minPressure.max;
      newHighestPressure[leg][sensor] = minPressure.max;
      newLowestPressure[leg][sensor] = maxPressure;
      lowestPressure[leg][sensor] = 0.f;
    }
    originalHasPressure[leg] = 0;
  }

  maxFootPressureCurrent = minPressure.max;
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
    theFsrData.legInfo[leg].hasPressure = footPressure > minPressure.max ? theFrameInfo.time : originalHasPressure[leg];
    originalHasPressure[leg] = theFsrData.legInfo[leg].hasPressure;
    theFsrData.legInfo[leg].backwardPressure = backwardPressure > minPressure.max * minSingleFSRPressureForPredictedSwitchFactor ? theFrameInfo.time : theFsrData.legInfo[leg].backwardPressure;
    theFsrData.legInfo[leg].forwardPressure = forwardPressure > minPressure.max * minSingleFSRPressureForPredictedSwitchFactor ? theFrameInfo.time : theFsrData.legInfo[leg].forwardPressure;
    theFsrData.legInfo[leg].leftPressure = leftPressure > minPressure.max * minSingleFSRPressureForPredictedSwitchFactor ? theFrameInfo.time : theFsrData.legInfo[leg].leftPressure;
    theFsrData.legInfo[leg].rightPressure = rightPressure > minPressure.max * minSingleFSRPressureForPredictedSwitchFactor ? theFrameInfo.time : theFsrData.legInfo[leg].rightPressure;
    theFsrData.legInfo[leg].hasPressureSince = theFsrData.legInfo[leg].hasPressure == theFrameInfo.time && originalHasPressure[leg] != theFsrData.lastUpdateTimestamp ? theFsrData.legInfo[leg].hasPressure : theFsrData.legInfo[leg].hasPressureSince;
    theFsrData.legInfo[leg].sagittalRatio = backwardPressure + forwardPressure > 0.f ? forwardPressure / (backwardPressure + forwardPressure) : 0.f;
    theFsrData.legInfo[leg].lateralRatio = leftPressure + rightPressure > 0.f ? leftPressure / (leftPressure + rightPressure) : 0.f;

    // Handling in case FSRs might need a new calibration
    if(theFsrData.legInfo[leg].hasPressure != theFrameInfo.time)
      theFsrData.legInfo[leg].hasPressure = footPressure > mapToRange(static_cast<float>(theFrameInfo.getTimeSince(theFsrData.legInfo[leg].hasPressureSince)), badMinPressureTimewindow.min, badMinPressureTimewindow.max, minPressure.max, minPressure.min) ? theFrameInfo.time : theFsrData.legInfo[leg].hasPressure;
  }

  if(!theMotionInfo.isMotion(MotionPhase::walk)
     || theFrameInfo.getTimeSince(theFsrData.legInfo[Legs::left].hasPressure) > maxTimeLegNoPressureForCalibration
     || theFrameInfo.getTimeSince(theFsrData.legInfo[Legs::right].hasPressure) > maxTimeLegNoPressureForCalibration)
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
    minPressure.max = (minPressureRange.max - minPressureRange.min) * factor + minPressureRange.min;
    minPressurePercent = minPressure.max / maxFootSumPressure;
    theFsrData.minPressure = minPressurePercent;
    supportSwitchCounter = 0;
    maxFootSumPressure = 0;
    theFsrData.isCalibrated = true;
  }

  theFsrData.lastUpdateTimestamp = theFrameInfo.time;
}
