/**
 * @file JointCalibration.h
 * Declaration of a struct for representing the calibration values of joints.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Math/BHMath.h"
#include "Tools/RobotParts/Joints.h"
#include "Tools/Streams/EnumIndexedArray.h"

STREAMABLE(JointCalibration,
{
  JointCalibration(),

  (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) offsets, /**< Information on the calibration of all joints. */
});

inline JointCalibration::JointCalibration()
{
  offsets.fill(0_deg);
}
