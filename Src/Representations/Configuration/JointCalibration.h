/**
 * @file JointCalibration.h
 * Declaration of a struct for representing the calibration values of joints.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Math/BHMath.h"
#include "Tools/RobotParts/Joints.h"
#include "Tools/Streams/EnumIndexedArray.h"

STREAMABLE(JointCalibration,
{
  STREAMABLE(JointInfo,
  {
    bool isMissing() const,

    (Angle)(0_deg) offset, /**< An offset added to the angle. */
    (Angle)(150_deg) minAngle, /** the minmal angle */
    (Angle)(150_deg) maxAngle, /** the maximal angle */
  }),

  (ENUM_INDEXED_ARRAY(JointInfo, (Joints) Joint)) joints, /**< Information on the calibration of all joints. */
});

inline bool JointCalibration::JointInfo::isMissing() const
{
  return minAngle >= maxAngle;
}
