/**
 * @file BHumanMessage.h
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Tools/Communication/BHumanTeamMessageParts/BSPLStandardMessage.h"
#include "Tools/Communication/BHumanTeamMessageParts/BHumanStandardMessage.h"
#include "Tools/Communication/BHumanTeamMessageParts/BHumanArbitraryMessage.h"
#include "Tools/Function.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * A representation of SPLStandardMessage a B-Human robot would send.
 */
STREAMABLE(BHumanMessage,
{
  virtual unsigned toLocalTimestamp(unsigned remoteTimestamp) const,

  (unsigned) timestamp,
  (BSPLStandardMessage) theBSPLStandardMessage,
  (BHumanStandardMessage) theBHumanStandardMessage,
  (BHumanArbitraryMessage) theBHumanArbitraryMessage,
});

inline unsigned BHumanMessage::toLocalTimestamp(unsigned) const { return 0u; }

/**
 * A struct that can generate the BHumanMessage,
 *     that this robot wants to send to its teammates.
 */
STREAMABLE_WITH_BASE(BHumanMessageOutputGenerator, BHumanMessage,
{
  FUNCTION(void(RoboCup::SPLStandardMessage* const m)) generate,

  (unsigned)(0) sentMessages,  //< count of sent messages
  (bool)(false) sendThisFrame, //< should send this frame -> is allowed to send this frame considering the spl rules
});
