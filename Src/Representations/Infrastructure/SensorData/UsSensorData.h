#pragma once

#include "Tools/Streams/AutoStreamable.h"

#include <array>

STREAMABLE(UsSensorData,
{
  // Inserted dummies, due to access based on array index. (http://www.aldebaran-robotics.com/documentation/naoqi/sensors/dcm/pref_file_architecture.html#us-actuator-value)
  ENUM(UsActuatorMode,
  {,
    leftToLeft,
    leftToRight,
    rightToLeft,
    rightToRight,
    numOfSingleUsActuatorModes,
    bothToSame = numOfSingleUsActuatorModes,
    bothToOther,
  });

  UsSensorData();
  ,
  (std::array<float, 10>) left, /**< up to ten measurements of the left sensor (in mm) */
  (std::array<float, 10>) right, /**< up to ten measurements of the right sensor (in mm) */
  (UsActuatorMode)(leftToLeft) actuatorMode, /**< The ultrasonice measure method which was used for measuring \c data[usL] and \c data[usR]. */
  (unsigned)(0) timeStamp, /**< The time when the ultrasonic measurements were taken. */
});

inline UsSensorData::UsSensorData()
{
  left.fill(0);
  right.fill(0);
}