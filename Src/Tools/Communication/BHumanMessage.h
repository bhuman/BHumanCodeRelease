/**
 * @file BHumanMessage.h
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Tools/Communication/BHumanTeamMessageParts/BSPLStandardMessage.h"
#include "Tools/Communication/BHumanTeamMessageParts/BHumanStandardMessage.h"
#include "Tools/Communication/BHumanTeamMessageParts/BHumanArbitraryMessage.h"

/**
 * A representation of SPLStandardMessage a B-Human robot would send.
 */
STREAMABLE(BHumanMessage,
{
  virtual unsigned toLocalTimestamp(unsigned remoteTimestamp) const,

  (BSPLStandardMessage) theBSPLStandardMessage,
  (BHumanStandardMessage) theBHumanStandardMessage,
  (BHumanArbitraryMessage) theBHumanArbitraryMessage,
});

inline unsigned BHumanMessage::toLocalTimestamp(unsigned) const { return 0u; }
