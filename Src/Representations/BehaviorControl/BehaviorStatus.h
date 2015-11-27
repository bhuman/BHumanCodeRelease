/**
 * @file Representations/BehaviorControl/BehaviorStatus.h
 * The file declares a struct that containts data about the current behavior state.
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/BehaviorControl/Role.h"

/**
 * @struct BehaviorStatus
 * A struct that containts data about the current behavior state.
 */
STREAMABLE(BehaviorStatus,
{,
  ((Role) RoleType)(Role::striker) role,
});
