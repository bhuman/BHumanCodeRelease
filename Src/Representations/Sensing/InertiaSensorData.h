/**
* @file InertiaSensorData.h
* Declaration of class InertiaSensorData.
* @author Colin Graf
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Math/Vector3.h"
#include "Representations/Infrastructure/SensorData.h"

/**
* @class InertiaSensorData
* Encapsulates inertia sensor data.
*/
STREAMABLE(InertiaSensorData,
{
public:
  enum {off = SensorData::off}, /**< A special value to indicate that the sensor is missing. */

  (Vector2<float>) gyro, /**< The change in orientation around the x- and y-axis. (in radian/s) */
  (Vector3<float>) acc, /**< The acceleration along the x-, y- and z-axis. (in m/s^2) */
  (bool)(false) calibrated, /**< Whether the inertia sensors are calibrated or not */
});
