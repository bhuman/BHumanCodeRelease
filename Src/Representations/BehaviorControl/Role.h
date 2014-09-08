/**
* @file Representations/BehaviorControl/Role.h
*
* Declaration of the the representation of a robot's behavior role
*
* @author Tim Laue
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

/**
* @class Role
* Representation of a robot's behavior role
*/
STREAMABLE(Role,
{
public:
  /** The different roles */
  ENUM(RoleType,
    undefined,
    keeper,
    striker,
    defender,
    none
  );

  /** Draws the current role next to the robot on the field view (in local robot coordinates)*/
  void draw() const,

  (RoleType) (striker) role,
});
