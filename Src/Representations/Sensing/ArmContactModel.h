/**
 * @file ArmContactModel.h
 *
 * Declaration of struct ArmContactModel.
 * @author <a href="mailto:fynn@informatik.uni-bremen.de">Fynn Feldpausch</a>
 * @author <a href="mailto:simont@informatik.uni-bremen.de">Simon Taddiken</a>
 * @author <a href="mailto:arneboe@informatik.uni-bremen.de">Arne Böckmann</a>
 * @author <a href="mailto:lrust@uni-bremen.de">Lukas Rust</a>
 *
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include <map>

STREAMABLE(ArmContactModel,
{
  ENUM(PushDirection,
  {,
    forward,
    backward,
    left,
    right,
    none,
  });

  STREAMABLE(ArmContact,
  {,
    (bool)(false) contact, /**< The contact state of the robot's arm. */
    (unsigned)(0) duration, /**< The duration of the push in motion frames. */
    (ENUM_INDEXED_ARRAY(bool, ArmContactModel::PushDirection)) directionMap,
    (PushDirection)(PushDirection::none) pushDirection, /**< direction in which the arm is being pushed */
    (bool)(false) armOnBack,
    (unsigned)(0) timeOfLastContact,
  }),

  (ArmContact[Arms::numOfArms]) status,
});
