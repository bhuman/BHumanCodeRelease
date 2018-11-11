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
 */

#include "FootSupportProvider.h"

MAKE_MODULE(FootSupportProvider, sensing)

FootSupportProvider::FootSupportProvider()
{
  weights[Legs::left][FsrSensors::fl] = weights[Legs::left][FsrSensors::bl] = outerWeight;
  weights[Legs::right][FsrSensors::fr] = weights[Legs::right][FsrSensors::br] = -outerWeight;
  weights[Legs::left][FsrSensors::fr] = weights[Legs::left][FsrSensors::br] = innerWeight;
  weights[Legs::right][FsrSensors::fl] = weights[Legs::right][FsrSensors::bl] = -innerWeight;

  FOREACH_ENUM(Legs::Leg, leg)
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
      highestPressure[leg][sensor] = minPressure;
}

void FootSupportProvider::update(FootSupport& theFootSupport)
{
  float totalPressure = 0.f;
  float weightedSum = 0.f;

  FOREACH_ENUM(Legs::Leg, leg)
    FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
    {
      float pressure = std::min(maxPressure, theFsrSensorData.pressures[leg][sensor]);
      highestPressure[leg][sensor] = std::max(highestPressure[leg][sensor], pressure);
      pressure /= highestPressure[leg][sensor];
      totalPressure += pressure;
      weightedSum += weights[leg][sensor] * pressure;
    }

  if(std::abs(totalPressure) > 0.f)
  {
    float prevSupport = theFootSupport.support;
    theFootSupport.support = weightedSum / totalPressure;
    theFootSupport.switched = prevSupport * theFootSupport.support < 0.f;
  }
  else
  {
    theFootSupport.support = 0.f;
    theFootSupport.switched = false;
  }
}
