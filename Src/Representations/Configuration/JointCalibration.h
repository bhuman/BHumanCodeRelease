/**
 * @file JointCalibration.h
 * Declaration of a struct for representing the calibration values of joints.
 * @author Thomas Röfer
 */

#pragma once

#include "Math/BHMath.h"
#include "RobotParts/Joints.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(JointCalibration,
{
  JointCalibration(),

  (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) offsets, /**< Information on the calibration of all joints. */
});

inline JointCalibration::JointCalibration()
{
  offsets.fill(0_deg);
}
