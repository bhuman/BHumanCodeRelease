/**
 * @file ArmContactModel.h
 *
 * Declaration of struct ArmContactModel.
 * @author <a href="mailto:fynn@informatik.uni-bremen.de">Fynn Feldpausch</a>
 * @author <a href="mailto:simont@informatik.uni-bremen.de">Simon Taddiken</a>
 * @author <a href="mailto:arneboe@informatik.uni-bremen.de">Arne Böckmann</a>
 *
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct ArmContactModel
 */
STREAMABLE(ArmContactModel,
{
  ENUM(PushDirection,
  {,
    N,    /**< Arm is being pushed to the front */
    S,    /**< Arm is being pushed to the back */
    W,    /**< Arm is being pushed to the left */
    E,    /**< Arm is being pushed to the right */
    NW,   /**< Arm is being pushed to the front and the left */
    NE,   /**< Arm is being pushed to the front and the right */
    SW,   /**< Arm is being pushed to the back and the left */
    SE,   /**< Arm is being pushed to the back and the right */
    NONE, /**< If no contact is detected */
  }),

  (bool)(false) contactLeft, /**< The contact state of the robot's left arm. */
  (bool)(false) contactRight, /**< The contact state of the robot's right arm. */

  // only evaluate these values if contactLeft or contactRight is true */
  (PushDirection)(NONE) pushDirectionLeft, /**< direction in which the left arm is being pushed */
  (PushDirection)(NONE) pushDirectionRight, /**< direction in which the right arm is being pushed */

  // only evaluate these values if contactLeft or contactRight is true
  (PushDirection)(NONE) lastPushDirectionLeft, /**< direction in which the left arm was last pushed */
  (PushDirection)(NONE) lastPushDirectionRight, /**< direction in which the right arm was last pushed */

  // The duration of the push in motion frames (100fps = 1s).
  (unsigned)(0) durationLeft,
  (unsigned)(0) durationRight,

  (unsigned)(0) timeOfLastContactLeft,
  (unsigned)(0) timeOfLastContactRight,
});
