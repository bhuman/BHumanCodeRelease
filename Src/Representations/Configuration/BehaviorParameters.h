/**
 * @file Representations/Configuration/BehaviorParameters/BehaviorParameters.h
 *
 * The file declares a struct that contains frequently used parameters of the behavior which can be modified.
 *
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/BehaviorControl/Role.h"
#include "Tools/Math/Angle.h"

STREAMABLE(BehaviorParameters,
{,
  (bool)(false) enableWalkStraight,
  (int)(7000) ballNotSeenTimeOut,
});
