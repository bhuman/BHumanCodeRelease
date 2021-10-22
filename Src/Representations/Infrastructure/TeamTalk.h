/**
 * @file TeamTalk.h
 *
 * Declares a representation for transmitting sound playback requests to the whole team
 *
 * @author Jan Blumenkamp
 */

#pragma once

#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * Information about what the team should say
 */
STREAMABLE(TeamTalk, COMMA public BHumanCompressedMessageParticle<TeamTalk>
{,
  (char)(64) say,
  (unsigned int)(0) timestamp,
});
