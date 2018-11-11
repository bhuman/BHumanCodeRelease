/**
 * @file Whistle.h
 *
 * Identified whistle sound
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Platform/Time.h"

#include <limits>

static_assert(std::numeric_limits<char>::is_signed, "This code expects that char is signed.");

STREAMABLE(Whistle, COMMA public BHumanMessageParticle<idWhistle>
{
  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override,

  (char)(0)         confidenceOfLastWhistleDetection, /**< Confidence based on hearing capability */
  (unsigned int)(0) lastTimeWhistleDetected,          /**< Timestamp */
});

inline void Whistle::operator>>(BHumanMessage& m) const
{
  m.theBHumanStandardMessage.lastTimeWhistleDetected = lastTimeWhistleDetected;
  m.theBHumanStandardMessage.confidenceOfLastWhistleDetection = confidenceOfLastWhistleDetection < 0 ? static_cast<char>(-1) : confidenceOfLastWhistleDetection;
}

inline void Whistle::operator<<(const BHumanMessage& m)
{
  if(m.hasBHumanParts)
  {
    lastTimeWhistleDetected = m.toLocalTimestamp(m.theBHumanStandardMessage.lastTimeWhistleDetected);
    confidenceOfLastWhistleDetection = m.theBHumanStandardMessage.confidenceOfLastWhistleDetection;
  }
  else
    confidenceOfLastWhistleDetection = -1;
}
