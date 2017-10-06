/**
 * @file Representations/Sensing/FootGroundContactState.h
 * @author Alexis Tsogias
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/RobotParts/FsrSensors.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Streams/EnumIndexedArray.h"

STREAMABLE(FootGroundContactState,
{
  ENUM(Foot,
  {,
    none,
    left,
    right,
    both,
  });

  FootGroundContactState(),

  (Foot)(none) contact,
  (ENUM_INDEXED_ARRAY(bool, (FsrSensors) FsrSensor)) leftSensorContacts, /**< Contact boolean for each sonsor in the left foot. */
  (ENUM_INDEXED_ARRAY(bool, (FsrSensors) FsrSensor)) rightSensorContacts, /**< Contact boolean for each sonsor in the right foot. */
  (ENUM_INDEXED_ARRAY(Vector2f, (Legs) Leg)) centerOfPressure, /**< The center of pressures for both feet, relative to their coordinate system. Undefined if no contact was detected for a foot. */
  (ENUM_INDEXED_ARRAY(float, (Legs) Leg)) relativeMassDistribution, /**< The distribution of the total mass per foot. */
  (ENUM_INDEXED_ARRAY(float, Foot)) totalMass, /**< The masses (in kg) per foot and for both feet. */
  (float)(0.f) maxTotal, /**< An upper bound for the masses measured. */
});

inline FootGroundContactState::FootGroundContactState()
{
  leftSensorContacts.fill(false);
  rightSensorContacts.fill(false);
  centerOfPressure.fill(Vector2f::Zero());
  relativeMassDistribution.fill(0.f);
  totalMass.fill(0.f);
}
