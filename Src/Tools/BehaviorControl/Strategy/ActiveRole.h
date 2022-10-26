/**
 * @file ActiveRole.h
 *
 * This file declares a base class for active roles.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Role.h"
#include "Streaming/Enum.h"

class ActiveRole : public Role
{
public:
  ENUM(Type,
  {,
    playBall,
    freeKickWall,
    closestToTeammatesBall,
  });

  static Role::Type toRole(Type type)
  {
    return static_cast<Role::Type>(activeRoleBegin + static_cast<unsigned>(type));
  }
};
