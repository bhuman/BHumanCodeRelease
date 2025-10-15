/*
 * @file ExtendedFootSupportProvider.cpp
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

#include "ExtendedFootSupportProvider.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include <cmath>

MAKE_MODULE(ExtendedFootSupportProvider);

ExtendedFootSupportProvider::ExtendedFootSupportProvider()
{
  weights[Legs::left][FsrSensors::fl] = weights[Legs::left][FsrSensors::bl] = outerWeight;
  weights[Legs::right][FsrSensors::fr] = weights[Legs::right][FsrSensors::br] = -outerWeight;
  weights[Legs::left][FsrSensors::fr] = weights[Legs::left][FsrSensors::br] = innerWeight;
  weights[Legs::right][FsrSensors::fl] = weights[Legs::right][FsrSensors::bl] = -innerWeight;

  lastSupportWithPressure = 0.f;
  lastSupport = 0;
  lastLegCurrent[Legs::left] = lastLegCurrent[Legs::right] = 0.f;
}

void ExtendedFootSupportProvider::update(FootSupport& theFootSupport)
{
  DECLARE_PLOT("module:ExtendedFootSupportProvider:discountFactor");

  if(theFrameInfo.time == 0)
    return;
  float totalPressure = 0.f;
  float weightedSum = 0.f;

  float predictedLegCurrentSum[Legs::numOfLegs];
  float legCurrentSum = 0.f;
  // Calculate the current pressure difference between both feet
  FOREACH_ENUM(Legs::Leg, leg)
  {
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
    {
      const float weight = weights[leg][sensor] * theSolePressureState.pressures[leg][sensor];
      weightedSum += weight;
      totalPressure += std::abs(weight);
    }
    predictedLegCurrentSum[leg] = theFilteredCurrent.legPitchCurrentSums[leg] +
                                  (theFilteredCurrent.legPitchCurrentSums[leg] - lastLegCurrent[leg]) * currentSumPredictionFactor;
    legCurrentSum += predictedLegCurrentSum[leg];
  }

  const float discountFactor = SystemCall::getMode() == SystemCall::simulatedRobot || legCurrentSum == 0 ? discountFactorRange.min :
                               mapToRange((lastSupport > 0.f ? predictedLegCurrentSum[Legs::right] : predictedLegCurrentSum[Legs::left]), currentRatioRange.min, currentRatioRange.max, discountFactorRange.min, discountFactorRange.max);
  PLOT("module:ExtendedFootSupportProvider:discountFactor", discountFactor);

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

    theFootSupport.predictedSwitched = theSolePressureState.isCalibrated // Min pressure must be calibrated first
                                       && predictedSupport * theFootSupport.support < discountFactor * discountFactor // support foot will switch
                                       && (theFootSupport.support - lastSupport) * theFootSupport.support < 0.f // direction change is towards 0
                                       && std::abs(theFootSupport.support) < discountFactorRange.max;
    lastSupport = theFootSupport.support;
  }
  else
  {
    theFootSupport.support = 0.f;
    theFootSupport.trustedSupport = false;
    theFootSupport.switched = false;
    theFootSupport.predictedSwitched = false;
  }

  lastLegCurrent[Legs::left] = theFilteredCurrent.legPitchCurrentSums[Legs::left];
  lastLegCurrent[Legs::right] = theFilteredCurrent.legPitchCurrentSums[Legs::right];
}
