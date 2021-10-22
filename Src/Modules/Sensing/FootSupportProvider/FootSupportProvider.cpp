/*
 * @file FootSupportProvider.cpp
 *
 * This file implements a module that provides an abstract distribution of
 * how much each foot supports the weight of the robot.
 *
 * The code is based on parts of the class BodyModel from the code of the
 * team UNSW Australia.
 *
 * @author Thomas Röfer
 * @author Philip Reichenberg
 */

#include "FootSupportProvider.h"
#include "Platform/File.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include <cmath>

MAKE_MODULE(FootSupportProvider, sensing);

FootSupportProvider::FootSupportProvider()
{
  weights[Legs::left][FsrSensors::fl] = weights[Legs::left][FsrSensors::bl] = outerWeight;
  weights[Legs::right][FsrSensors::fr] = weights[Legs::right][FsrSensors::br] = -outerWeight;
  weights[Legs::left][FsrSensors::fr] = weights[Legs::left][FsrSensors::br] = innerWeight;
  weights[Legs::right][FsrSensors::fl] = weights[Legs::right][FsrSensors::bl] = -innerWeight;

  FOREACH_ENUM(Legs::Leg, leg)
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
    {
      highestPressure[leg][sensor] = minPressure;
      newHighestPressure[leg][sensor] = minPressure;
      newLowestPressure[leg][sensor] = maxPressure;
    }

  updatePressureTimestamp = 0;
  supportSwitchCounter = 0;
  lastSupportSwitch = 0;
  lastSupportWithPressure = 0.f;
  maxFootPressureCurrent = 0.f;
  maxFootSumPressure = 0.f;
  lastSupport = 0;
}

void FootSupportProvider::update(FootSupport& theFootSupport)
{
  DECLARE_PLOT("module:FootSupportProvider:supportSwitch");
  DECLARE_PLOT("module:FootSupportProvider:supportPredictSwitch");
  if(theFrameInfo.time == 0)
    return;
  float totalPressure = 0.f;
  float weightedSum = 0.f;
  float filteredPressure[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Calibrated pressure. Measured pressure minus min pressure. */

  // Calculate the current pressure difference between both feet
  FOREACH_ENUM(Legs::Leg, leg)
  {
    float footPressure = 0.f;
    float backwardPressure = 0.f;
    float forwardPressure = 0.f;
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
    {
      float pressure = std::max(0.f, std::min(maxPressure, theFsrSensorData.pressures[leg][sensor]) - lowestPressure[leg][sensor]);
      filteredPressure[leg][sensor] = pressure;
      footPressure += pressure;
      if(sensor == FsrSensors::bl || sensor == FsrSensors::br)
        backwardPressure += pressure;
      else
        forwardPressure += pressure;
      highestPressure[leg][sensor] = std::max(highestPressure[leg][sensor], pressure);
      newHighestPressure[leg][sensor] = std::max(newHighestPressure[leg][sensor], pressure);
      newLowestPressure[leg][sensor] = std::min(newLowestPressure[leg][sensor], pressure);
      pressure /= highestPressure[leg][sensor];
      totalPressure += pressure;
      weightedSum += weights[leg][sensor] * pressure;
    }
    // Update last pressure time stamps
    const unsigned lastHasPressure = theFootSupport.footPressure[leg].hasPressure;
    theFootSupport.footPressure[leg].hasPressure = footPressure > minPressure ? theFrameInfo.time : theFootSupport.footPressure[leg].hasPressure;
    theFootSupport.footPressure[leg].backwardPressure = backwardPressure > minPressure ? theFrameInfo.time : theFootSupport.footPressure[leg].backwardPressure;
    theFootSupport.footPressure[leg].forwardPressure = forwardPressure > minPressure ? theFrameInfo.time : theFootSupport.footPressure[leg].forwardPressure;
    theFootSupport.footPressure[leg].hasPressureSince = theFootSupport.footPressure[leg].hasPressure == theFrameInfo.time && lastHasPressure != theFootSupport.timeSinceLastUpdate ? theFootSupport.footPressure[leg].hasPressure : theFootSupport.footPressure[leg].hasPressureSince;
    if(Legs::left == leg)
      leftFootPressureBuffer.push_front(footPressure > minPressure);
    else
      rightFootPressureBuffer.push_front(footPressure > minPressure);
    maxFootPressureCurrent = std::max(maxFootPressureCurrent, footPressure);
  }

  // Update footSupport state
  if(std::abs(totalPressure) > 0.f)
  {
    lastSupport = theFootSupport.support;
    float prevSupport = theFootSupport.support;
    theFootSupport.support = weightedSum / totalPressure;
    theFootSupport.switched = (lastSupportWithPressure * theFootSupport.support < 0.f || (prevSupport == 0.f && theFootSupport.support != 0.f)) // switched base on sign
                              && ((theFootSupport.footPressure[Legs::left].hasPressure == theFrameInfo.time && theFootSupport.support > 0.f) || // new support foot has pressure
                                  (theFootSupport.footPressure[Legs::right].hasPressure == theFrameInfo.time && theFootSupport.support < 0.f));

    // save the last measurement, when the feet had enough pressure
    lastSupportWithPressure = ((theFootSupport.footPressure[Legs::left].hasPressure == theFrameInfo.time && theFootSupport.support > 0.f) || // new support foot has pressure
                               (theFootSupport.footPressure[Legs::right].hasPressure == theFrameInfo.time && theFootSupport.support < 0.f)) ? theFootSupport.support : lastSupportWithPressure;
    if(theFootSupport.switched)
      theFootSupport.lastSwitch = theFrameInfo.time - lastSupportSwitch;

    float predictedSupport = theFootSupport.support + 3.f * (theFootSupport.support - lastSupport); //current vel

    const float minSinglePressure = minSingleFSRPressureForPredictedSwitchFactor * minPressure;
    bool leftFSR = filteredPressure[Legs::left][FsrSensors::fl] + filteredPressure[Legs::left][FsrSensors::fr] > minSinglePressure
                   || filteredPressure[Legs::left][FsrSensors::bl] + filteredPressure[Legs::left][FsrSensors::br] > minSinglePressure;
    bool rightFSR = filteredPressure[Legs::right][FsrSensors::fl] + filteredPressure[Legs::right][FsrSensors::fr] > minSinglePressure
                    || filteredPressure[Legs::right][FsrSensors::bl] + filteredPressure[Legs::right][FsrSensors::br] > minSinglePressure;

    theFootSupport.predictedSwitched = theFootSupport.isCalibrated // Min pressure must be calibrated first
                                       && predictedSupport * theFootSupport.support < 0.f // support foot will switch
                                       && ((theFootSupport.support < 0.f && leftFSR) // new support foot has some pressure and had not have it before
                                           || (theFootSupport.support > 0.f && rightFSR));
  }
  else
  {
    theFootSupport.support = 0.f;
    theFootSupport.switched = false;
    theFootSupport.predictedSwitched = false;
  }
  // Update the heighest pressures
  if(theFrameInfo.getTimeSince(updatePressureTimestamp) > highestPressureUpdateTime && theFootSupport.switched)
  {
    updatePressureTimestamp = theFrameInfo.time;
    FOREACH_ENUM(Legs::Leg, leg)
      FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
      {
        highestPressure[leg][sensor] = newHighestPressure[leg][sensor];
        newHighestPressure[leg][sensor] = minPressure;
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

  // Enough support switches happened. Update the lowest measured pressures.
  if(supportSwitchCounter >= numOfSupportSwitches)
  {
    // Calibrate lowestPressure offsets
    FOREACH_ENUM(Legs::Leg, leg)
      FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
      {
        lowestPressure[leg][sensor] = newLowestPressure[leg][sensor];
        newLowestPressure[leg][sensor] = maxPressure;
        if(lowestPressure[leg][sensor] > 0.01f)
          OUTPUT_TEXT("fsr" << TypeRegistry::getEnumName(leg) << " " << TypeRegistry::getEnumName(sensor) << " has offsets of " << lowestPressure[leg][sensor]);
      }

    maxFootSumPressure /= supportSwitchCounter;
    const float factor = Rangef::ZeroOneRange().limit((maxFootSumPressure - minPressureInterpolationValues.min) / (minPressureInterpolationValues.max - minPressureInterpolationValues.min));
    minPressure = (minPressureRange.max - minPressureRange.min) * factor + minPressureRange.min;
    maxFootSumPressure = 0.f;
    supportSwitchCounter = 0;
    theFootSupport.isCalibrated = true;
  }

  theFootSupport.minPressure = minPressure;
  theFootSupport.timeSinceLastUpdate = theFrameInfo.time;

  PLOT("module:FootSupportProvider:supportPredictSwitch", theFootSupport.predictedSwitched ? 1.f : 0.f);
  PLOT("module:FootSupportProvider:supportSwitch", theFootSupport.switched ? 1.f : 0.f);
}
