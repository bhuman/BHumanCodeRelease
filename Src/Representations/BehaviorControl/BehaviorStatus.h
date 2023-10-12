/**
 * @file Representations/BehaviorControl/BehaviorStatus.h
 * The file declares a struct that contains data about the current behavior state.
 * @author Andreas Stolpmann
 */

#pragma once

#include "Tools/Communication/BHumanMessageParticle.h"
#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

/**
 * @struct BehaviorStatus
 * A struct that contains data about the current behavior state.
 */
STREAMABLE(BehaviorStatus, COMMA public BHumanCompressedMessageParticle<BehaviorStatus>
{,
  (bool)(false) calibrationFinished, /**< Set when the behavior is done with calibration and wants to transition to the unstiff state. */
  (int)(-1) passTarget, /**< The number of the passed-to player. */
  (int)(-1) passOrigin, /**< The number of the player that passes to this one */
  (Vector2f)(Vector2f::Zero()) walkingTo, /**< The target position the robot is walking to (in robot relative coordinates). */
  (float) speed, /**< The absolute speed in mm/s. */
  (std::optional<Vector2f>) shootingTo, /**< The target position the robot is kicking the ball to (in robot relative coordinates). */
});
