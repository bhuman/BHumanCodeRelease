/**
* @file Representations/BehaviorControl/BehaviorStatus.h
* The file declares a class that containts data about the current behavior state.
* @author Andreas Stolpmann
*/

#pragma once
#include "Tools/Streams/AutoStreamable.h"

/**
* @class BehaviorStatus
* A class that containts data about the current behavior state.
*/
STREAMABLE(BehaviorStatus,
{
public:
  ENUM(Role,
    striker,
    defender,
    keeper
  );

  ENUM(TeamColor,
    red,
    blue
  );
  ,
  (TeamColor)(red) teamColor,
  (Role)(striker) role,
});
