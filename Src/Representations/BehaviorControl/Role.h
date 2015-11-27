/**
 * @file Representations/BehaviorControl/Role.h
 *
 * Declaration of the the representation of a robot's behavior role
 *
 * @author Tim Laue, Andreas Stolpmann
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

/**
 * @struct Role
 * Representation of a robot's behavior role
 */
STREAMABLE(Role,
{
  /** The different roles */
  ENUM(RoleType,
  {,
    undefined,
    keeper,
    striker,
    defender,
    supporter,
    none,
  });

  /** Draws the current role next to the robot on the field view (in local robot coordinates) */
  void draw() const,

  /** Instance of role */
  (RoleType)(striker) role,
});

STREAMABLE(TeammateRoles,
{
  Role::RoleType operator [] (const size_t i) const;
  Role::RoleType& operator [] (const size_t i);
  static const char* getName(Role::RoleType e),

  (std::vector<Role::RoleType>) roles,
});
