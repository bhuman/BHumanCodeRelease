/**
 * @file SideInformation.h
 *
 * Declaration of struct SideInformation.
 * The SPL field is symmetric and perceiving the field elements alone does not
 * allow to infer the correct playing direction. This struct contains some information
 * that might help to solve this problem in some situations.
 *
 * @author Tim Laue
 */

#pragma once

#include "Streaming/AutoStreamable.h"

/**
 * @struct SideInformation
 * Information to solve the field's symmetry
 */
STREAMABLE(SideInformation,
{
  /** Verifies that the representation contains valid values. */
  void verify() const,

  (bool)(false) robotMustBeInOwnHalf,            /**< The robot must still be on its own half of the field (as it did not walk far enough or due some special configuration [debugging / testing / demo feature / technical challenge]). */
  (bool)(false) robotMustBeInOpponentHalf,       /**< The robot must be in the opponent half of the field (debugging / testing / demo feature / technical challenge). */
  (float)(100000.f) largestXCoordinatePossible,  /**< The largest x-coordinate that the robot could have walked to. Actual x-coordinate is smaller. */
});
