/**
 * @file Representations/Configuration/BehaviorParameters/Behavior2015Parameters.h
 *
 * The file declares a struct that contains frequently used parameters of the behavior which can be modified.
 *
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/BehaviorControl/Role.h"
#include "Tools/Math/Angle.h"

STREAMABLE(Behavior2015Parameters,
{,
  (int)(7000) ballNotSeenTimeOut,
});
