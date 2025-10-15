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
#include <cmath>

MAKE_MODULE(FootSupportProvider);

FootSupportProvider::FootSupportProvider()
{
  weights[Legs::left][FsrSensors::fl] = weights[Legs::left][FsrSensors::bl] = outerWeight;
  weights[Legs::right][FsrSensors::fr] = weights[Legs::right][FsrSensors::br] = -outerWeight;
  weights[Legs::left][FsrSensors::fr] = weights[Legs::left][FsrSensors::br] = innerWeight;
  weights[Legs::right][FsrSensors::fl] = weights[Legs::right][FsrSensors::bl] = -innerWeight;

  lastSupportWithPressure = 0.f;
  lastSupport = 0;
}

void FootSupportProvider::update(FootSupport& theFootSupport)
{
  if(theFrameInfo.time == 0)
    return;
  float totalPressure = 0.f;
  float weightedSum = 0.f;

  // Calculate the current pressure difference between both feet
  FOREACH_ENUM(Legs::Leg, leg)
  {
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
    {
      const float weight = weights[leg][sensor] * theSolePressureState.pressures[leg][sensor];
      weightedSum += weight;
      totalPressure += std::abs(weight);
    }
  }

  // Update footSupport state
  if(totalPressure > theSolePressureState.minPressure)
  {
    theFootSupport.trustedSupport = theSolePressureState.isCalibrated;
    theFootSupport.support = weightedSum / totalPressure;
    theFootSupport.switched = (lastSupportWithPressure * theFootSupport.support < 0.f || (lastSupport == 0.f && theFootSupport.support != 0.f)) // switched base on sign
                              && ((theSolePressureState.legInfo[Legs::left].hasPressure && theFootSupport.support > 0.f) || // new support foot has pressure
                                  (theSolePressureState.legInfo[Legs::right].hasPressure && theFootSupport.support < 0.f));

    // save the last measurement, when the feet had enough pressure
    lastSupportWithPressure = ((theSolePressureState.legInfo[Legs::left].hasPressure && theFootSupport.support > 0.f) || // new support foot has pressure
                               (theSolePressureState.legInfo[Legs::right].hasPressure && theFootSupport.support < 0.f)) ? theFootSupport.support : lastSupportWithPressure;
    if(theFootSupport.switched)
    {
      theFootSupport.lastSwitch = theFrameInfo.time - lastSupportSwitch;
      lastSupportSwitch = theFrameInfo.time;
    }

    float predictedSupport = theFootSupport.support + predictionFactor * (theFootSupport.support - lastSupport); //current vel

    bool leftFSR = (theSolePressureState.pressures[Legs::left][FsrSensors::fl] + theSolePressureState.pressures[Legs::left][FsrSensors::fr]) * minSingleFSRPressureForPredictedSwitchFactor > theSolePressureState.minPressure
                   || (theSolePressureState.pressures[Legs::left][FsrSensors::bl] + theSolePressureState.pressures[Legs::left][FsrSensors::br]) * minSingleFSRPressureForPredictedSwitchFactor > theSolePressureState.minPressure;
    bool rightFSR = (theSolePressureState.pressures[Legs::right][FsrSensors::fl] + theSolePressureState.pressures[Legs::right][FsrSensors::fr]) * minSingleFSRPressureForPredictedSwitchFactor > theSolePressureState.minPressure
                    || (theSolePressureState.pressures[Legs::right][FsrSensors::bl] + theSolePressureState.pressures[Legs::right][FsrSensors::br]) * minSingleFSRPressureForPredictedSwitchFactor > theSolePressureState.minPressure;

    theFootSupport.predictedSwitched = theSolePressureState.isCalibrated // Min pressure must be calibrated first
                                       && predictedSupport * theFootSupport.support < 0.f // support foot will switch
                                       && ((theFootSupport.support < 0.f && leftFSR && theSolePressureState.legInfo[Legs::right].totals < currentSupportMaxPressure && theSolePressureState.legInfo[Legs::left].totals > currentSwingMinPressure) // new support foot has some pressure and had not have it before
                                           || (theFootSupport.support > 0.f && rightFSR && theSolePressureState.legInfo[Legs::left].totals < currentSupportMaxPressure && theSolePressureState.legInfo[Legs::right].totals > currentSwingMinPressure));

    lastSupport = theFootSupport.support;
  }
  else
  {
    theFootSupport.support = 0.f;
    theFootSupport.trustedSupport = false;
    theFootSupport.switched = false;
    theFootSupport.predictedSwitched = false;
  }
}
