/**
* @file Representations/BehaviorControl/BehaviorStatus.h
* The file declares a class that containts data about the current behavior state.
* @author Andreas Stolpmann
*/

#pragma once
#include "Representations/BehaviorControl/Role.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

/**
* @class BehaviorStatus
* A class that containts data about the current behavior state.
*/
STREAMABLE(BehaviorStatus,
{
public:
  ENUM(TeamColor,
    red,
    blue
  ),

  (Role, RoleType) (striker) role,
  (TeamColor) (red) teamColor,

  (float) (999999.f) estimatedTimeToReachBall,
});
