/**
 * @file IndirectKick.h
 *
 * Declaration of a representation for the indirect kick rule.
 *
 * @author Ingo Kelm
 */

#pragma once

#include "Math/Eigen.h"
#include "Framework/Settings.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"
#include "Tools/Communication/BHumanMessageParticle.h"

STREAMABLE(IndirectKick, COMMA public BHumanCompressedMessageParticle<IndirectKick>
{,
  // don't need to be communicated
  (bool)(true) allowDirectKick, /**< true if direct kicks are allowed */
  (bool)(false) sacAlternateTactic, /**< Play offensive in the shared autonomy challenge. */
  (unsigned int)(0) lastSetPlayTime, /**< timestamp of last gamestate change */
  (std::array<unsigned int, Settings::highestValidPlayerNumber - Settings::lowestValidPlayerNumber + 1>) lastBallContactTimestamps, /**< array of last deliberate ball contact for each player (index) */

  // need to be communicated
  (unsigned int)(0) lastKickTimestamp,
});
