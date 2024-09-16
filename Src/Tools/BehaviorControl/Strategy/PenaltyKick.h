/**
 * @file PenaltyKick.h
 *
 * This file declares the representation of a penalty kick.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "SetPlay.h"
#include "Tactic.h"
#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"
#include <vector>

STREAMABLE_WITH_BASE(PenaltyKick, SetPlay,
{,
});

STREAMABLE_WITH_BASE(OwnPenaltyKick, PenaltyKick,
{
  ENUM(Type,
  {,
    theOneTrueOwnPenaltyKick,
    theOneTrueOwnPenaltyKickAttacking,
  });

  static SetPlay::Type toSetPlay(Type type)
  {
    return static_cast<SetPlay::Type>(ownPenaltyKickBegin + static_cast<unsigned>(type));
  },
});

STREAMABLE_WITH_BASE(OpponentPenaltyKick, PenaltyKick,
{
  ENUM(Type,
  {,
    theOneTrueOpponentPenaltyKick,
  });

  static SetPlay::Type toSetPlay(Type type)
  {
    return static_cast<SetPlay::Type>(opponentPenaltyKickBegin + static_cast<unsigned>(type));
  },
});
