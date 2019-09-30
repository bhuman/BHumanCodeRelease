/**
 * @file TeamTalk.h
 *
 * Declares a representation for transmitting sound playback requests to the whole team
 *
 * @author Jan Blumenkamp
 */

#pragma once

#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * Information about what the team should say
 */
STREAMABLE(TeamTalk, COMMA public BHumanMessageParticle<idTeamTalk>
{
    /** BHumanMessageParticle functions */
    void operator>>(BHumanMessage& m) const override;
    void operator<<(const BHumanMessage& m) override,

    (char)(0) say,
    (unsigned int)(0) timestamp,
});

inline void TeamTalk::operator>>(BHumanMessage& m) const
{
  m.theBHumanStandardMessage.say = say;
  m.theBHumanStandardMessage.nextTeamTalk = timestamp;
}

inline void TeamTalk::operator<<(const BHumanMessage& m)
{
  if(!m.hasBHumanParts)
    return;
  say = m.theBHumanStandardMessage.say;
  timestamp = m.toLocalTimestamp(m.theBHumanStandardMessage.nextTeamTalk);
}
