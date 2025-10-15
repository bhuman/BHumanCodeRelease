/**
 * @file RobotStableState.h
 * This representation holds information of the CoM position in the foot area.
 * @author Philip Reichenberg
 */

#pragma once

#include "Math/Angle.h"
#include "Math/Eigen.h"
#include "Math/Rotation.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"
#include "RobotParts/Legs.h"
#include "Streaming/Function.h"

/**
 * The inertialData contains filtered data from the IMU.
 */

STREAMABLE(ComInFootState,
{,
  (float)(0.f) forward, /**< Forward position of the CoM [0..1]. */
  // The side is defined as follows:
  // 0 is in the coordinate y = 0 in the foot area.
  // 0.5 is the inner edge of the sole
  // 0.75 is the origin point of the sole
  // 1 is the outer edge of the sole
  (float)(0.f) outerSide, /**< Sideward position of the CoM [0..1]. */
  (float)(0.f) outerSideAbsolute, /**< Sideward position of the CoM in mm. */
});

STREAMABLE(RobotStableState,
{
  FUNCTION(ComInFootState(const Legs::Leg leg)) getTurnPoint;
  FUNCTION(void(const bool isLeftPhase, const bool prediction, const bool findTurnPoint)) predictRotation;
  void draw(),

  (std::optional<Vector3f>) comInFloor,
  (ENUM_INDEXED_ARRAY(ComInFootState, Legs::Leg)) comInTorso, /**< In robot coordinates the % positions. */
  (ENUM_INDEXED_ARRAY(ComInFootState, Legs::Leg)) comInFeet, /**< In feet coordinates the % positions. */
  (Vector2a) predictedTorsoRotation,
  (RotationMatrix) predictedTorsoRotationMatrix,
  (Vector3f)(Vector3f::Zero()) lightCenterOfMass, /**< Position of the CoM in robot coordinates, based on all limbs except the legs. */
  (unsigned int)(0) lastUpdate, /**< Timestamp of the last update. */
});
