/**
 * @file IndirectKick.h
 *
 * Declaration of a representation for the indirect kick rule.
 *
 * @author Ingo Kelm
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Tools/Communication/BHumanMessageParticle.h"

STREAMABLE(IndirectKick, COMMA public BHumanCompressedMessageParticle<IndirectKick>
{,
  // don't need to be communicated
  (bool)(true) allowDirectKick, /**< true if direct kicks are allowed */
  (bool)(true) oldAllowDirectKick, /**< previous value of allowDirectKick */
  (unsigned)(0) lastSetPlayTime, /**< timestamp of last game state change */
  (bool)(false) directKickAfterFreeKick, /**< true if a direct kick is allowed after a failed free kick */
  (int)(0) lastStriker,

  // need to be communicated
  (unsigned)(0) lastKickTimestamp,
});
