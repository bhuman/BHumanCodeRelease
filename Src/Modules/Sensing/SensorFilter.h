/**
* @file SensorFilter.h
* Declaration of module SensorFilter.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Configuration/DamageConfiguration.h"

MODULE(SensorFilter)
  REQUIRES(SensorData)
  REQUIRES(InertiaSensorData)
  REQUIRES(OrientationData)
  REQUIRES(DamageConfiguration)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FilteredSensorData)
END_MODULE

/**
* A module for sensor data filtering.
*/
class SensorFilter : public SensorFilterBase
{
public:
  /**
  * Updates the FilteredSensorData representation.
  * @param filteredSensorData The sensor data representation which is updated by this module.
  */
  void update(FilteredSensorData& filteredSensorData);

private:
  float gyroAngleXSum;
  unsigned lastIteration;
};
