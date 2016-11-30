/**
 * @file Representations/BehaviorControl/SPLStandardBehaviorStatus.h
 * The file declares a struct that contains those parts of the current behavior state
 * that are officially part of the SPLStandardMessage
 *
 * @author Tim Laue
 */

#pragma once
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include <cstdint>

/**
 * @struct SPLStandardBehaviorStatus
 * A struct that contains standard data about the current behavior state.
 */
STREAMABLE(SPLStandardBehaviorStatus,
{,
  (Vector2f)(Vector2f::Zero()) walkingTo,
  (Vector2f)(Vector2f::Zero()) shootingTo,
  (int8_t)(0) intention,
  (int16_t)(0) averageWalkSpeed,
  (int16_t)(0) maxKickDistance,
});
