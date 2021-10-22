/**
 * @file LibTeammates.h
 *
 * A representation that contains information about teammate positions
 *
 * @author Lukas Malte Monnerjahn
 */

#pragma once
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibTeammates,
{,
  (int) teammatesInOpponentPenaltyArea,     /**< How many teammates are in the opponents penalty area */
  (int) nonKeeperTeammatesInOwnPenaltyArea, /**< How many non keeper teammates are in the own penalty area */
});