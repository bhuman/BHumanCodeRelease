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
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"
#include "Platform/File.h"
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
  DECLARE_PLOT("module:FootSupportProvider:supportSwitch");
  DECLARE_PLOT("module:FootSupportProvider:supportPredictSwitch");
  DECLARE_PLOT("module:FootSupportProvider:support");
  if(theFrameInfo.time == 0)
    return;
  float totalPressure = 0.f;
  float weightedSum = 0.f;

  // Calculate the current pressure difference between both feet
  FOREACH_ENUM(Legs::Leg, leg)
  {
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
    {
      const float weight = weights[leg][sensor] * theFsrData.pressures[leg][sensor];
      weightedSum += weight;
      totalPressure += std::abs(weight);
    }
  }

  // Update footSupport state
  if(totalPressure > theFsrData.minPressure)
  {
    theFootSupport.trustedSupport = theFsrData.isCalibrated;
    theFootSupport.support = weightedSum / totalPressure;
    theFootSupport.switched = (lastSupportWithPressure * theFootSupport.support < 0.f || (lastSupport == 0.f && theFootSupport.support != 0.f)) // switched base on sign
                              && ((theFsrData.legInfo[Legs::left].hasPressure == theFrameInfo.time && theFootSupport.support > 0.f) || // new support foot has pressure
                                  (theFsrData.legInfo[Legs::right].hasPressure == theFrameInfo.time && theFootSupport.support < 0.f));

    // save the last measurement, when the feet had enough pressure
    lastSupportWithPressure = ((theFsrData.legInfo[Legs::left].hasPressure == theFrameInfo.time && theFootSupport.support > 0.f) || // new support foot has pressure
                               (theFsrData.legInfo[Legs::right].hasPressure == theFrameInfo.time && theFootSupport.support < 0.f)) ? theFootSupport.support : lastSupportWithPressure;
    if(theFootSupport.switched)
    {
      theFootSupport.lastSwitch = theFrameInfo.time - lastSupportSwitch;
      lastSupportSwitch = theFrameInfo.time;
    }

    float predictedSupport = theFootSupport.support + 3.f * (theFootSupport.support - lastSupport); //current vel

    bool leftFSR = theFsrData.legInfo[Legs::left].forwardPressure == theFsrData.lastUpdateTimestamp
                   || theFsrData.legInfo[Legs::left].backwardPressure == theFsrData.lastUpdateTimestamp;
    bool rightFSR = theFsrData.legInfo[Legs::right].forwardPressure == theFsrData.lastUpdateTimestamp
                    || theFsrData.legInfo[Legs::right].backwardPressure == theFsrData.lastUpdateTimestamp;

    theFootSupport.predictedSwitched = theFsrData.isCalibrated // Min pressure must be calibrated first
                                       && predictedSupport * theFootSupport.support < 0.f // support foot will switch
                                       && ((theFootSupport.support < 0.f && leftFSR && theFsrData.legInfo[Legs::right].totals < currentSupportMaxPressure && theFsrData.legInfo[Legs::left].totals > currentSwingMinPressure) // new support foot has some pressure and had not have it before
                                           || (theFootSupport.support > 0.f && rightFSR && theFsrData.legInfo[Legs::left].totals < currentSupportMaxPressure && theFsrData.legInfo[Legs::right].totals > currentSwingMinPressure));

    lastSupport = theFootSupport.support;
  }
  else
  {
    theFootSupport.support = 0.f;
    theFootSupport.trustedSupport = false;
    theFootSupport.switched = false;
    theFootSupport.predictedSwitched = false;
  }

  PLOT("module:FootSupportProvider:support", theFootSupport.support);

  PLOT("module:FootSupportProvider:supportPredictSwitch", theFootSupport.predictedSwitched ? 1.f : 0.f);
  PLOT("module:FootSupportProvider:supportSwitch", theFootSupport.switched ? 1.f : 0.f);
}
