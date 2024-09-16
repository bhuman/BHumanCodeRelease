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
{
  STREAMABLE(Condition,
  {,
    (bool)(false) nothingPrediction, /** Is the current kickoff prediction 'nothing'? */
  }),

  (Condition)preconditions, /**< The conditions under which the kickoff may be selected. */
  (Condition)invariants, /**< The conditions under which the kickoff can stay selected. */
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
    kiteKickOffensive,
  });

  static SetPlay::Type toSetPlay(Type type)
  {
    return static_cast<SetPlay::Type>(opponentKickOffBegin + static_cast<unsigned>(type));
  },
});
