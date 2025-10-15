/**
 * @file MassCalibration.h
 * Declaration of a struct for representing the relative positions and masses of mass points.
 * @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 */

#pragma once

#include "Math/Eigen.h"
#include "RobotParts/Limbs.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(MassCalibration,
{
  /**
   * Information on the mass distribution of a limb of the robot.
   */
  STREAMABLE(MassInfo,
  {
    Matrix3f inertiaMatrix = Matrix3f::Zero(); /**< The inertia matrix relative to the limb's origin (i.e. in the same coordinate system as \c offset is given, NOT relative to the \c offset) (in g*mm^2). */

    void onRead(),

    (float)(0.f) mass, /**< The mass of this limb (in g). */
    (Vector3f)(Vector3f::Zero()) offset, /**< The offset of the center of mass of this limb relative to its hinge. */
    (std::array<float, 6>) inertia, /**< The non-redundant entries of the inertia matrix in the order xx, xy, xz, yy, yz, zz (in g*mm^2). */
  });

  float totalMass = 0.f; /**< The total mass of the robot (in g). */

  void onRead(),

  (ENUM_INDEXED_ARRAY(MassInfo, Limbs::Limb)) masses, /**< Information on the mass distribution of all joints. */
});

inline void MassCalibration::MassInfo::onRead()
{
  inertiaMatrix << inertia[0], inertia[1], inertia[2], inertia[1], inertia[3], inertia[4], inertia[2], inertia[4], inertia[5];
}

inline void MassCalibration::onRead()
{
  totalMass = 0.f;
  for(int i = 0; i < Limbs::numOfLimbs; i++)
    totalMass += masses[i].mass;
}
