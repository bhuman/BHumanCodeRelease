/**
 * @file MassCalibration.h
 * Declaration of a struct for representing the relative positions and masses of mass points.
 * @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/RobotParts/Limbs.h"
#include "Tools/Streams/EnumIndexedArray.h"

STREAMABLE(MassCalibration,
{
  /**
   * Information on the mass distribution of a limb of the robot.
   */
  STREAMABLE(MassInfo,
  {,
    (float)(0.f) mass, /**< The mass of this limb (in g). */
    (Vector3f)(Vector3f::Zero()) offset, /**< The offset of the center of mass of this limb relative to its hinge. */
  });

  float totalMass = 0.f; /**< The total mass of the Robot (in g). */

  void onRead(),

  (ENUM_INDEXED_ARRAY(MassInfo, Limbs::Limb)) masses, /**< Information on the mass distribution of all joints. */
});

inline void MassCalibration::onRead()
{
  totalMass = 0.f;
  for(int i = 0; i < Limbs::numOfLimbs; i++)
    totalMass += masses[i].mass;
}
