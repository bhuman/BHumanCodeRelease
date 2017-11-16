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
#include "Tools/RobotParts/Arms.h"
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
  });

  STREAMABLE(ArmContact,
  {,
    (bool)(false) contact, /**< The contact state of the robot's arm. */

    // only evaluate these values if contact is true */
    ((ArmContactModel) PushDirection)(ArmContactModel::NONE) pushDirection, /**< direction in which the arm is being pushed */
    ((ArmContactModel) PushDirection)(ArmContactModel::NONE) lastPushDirection, /**< direction in which the arm was last pushed */

    // The duration of the push in motion frames (100fps = 1s).
    (unsigned)(0) duration,

    (unsigned)(0) timeOfLastContact,
  }),

  (ArmContact[Arms::numOfArms]) status,
});
