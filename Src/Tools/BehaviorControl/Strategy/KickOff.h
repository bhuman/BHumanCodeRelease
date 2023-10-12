/**
 * @file KickOff.h
 *
 * This file declares the representation of a kick-off.
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

STREAMABLE_WITH_BASE(KickOff, SetPlay,
{,
});

STREAMABLE_WITH_BASE(OwnKickOff, KickOff,
{
  ENUM(Type,
  {,
    directKickOff,
  });

  static SetPlay::Type toSetPlay(Type type)
  {
    return static_cast<SetPlay::Type>(ownKickOffBegin + static_cast<unsigned>(type));
  },
});

STREAMABLE_WITH_BASE(OpponentKickOff, KickOff,
{
  ENUM(Type,
  {,
    kiteKickOff,
  });

  static SetPlay::Type toSetPlay(Type type)
  {
    return static_cast<SetPlay::Type>(opponentKickOffBegin + static_cast<unsigned>(type));
  },
});
