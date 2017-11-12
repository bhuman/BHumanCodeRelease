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

STREAMABLE(Whistle, COMMA public BHumanMessageParticle<idWhistle>
{
  /** BHumanMessageParticle functions */
  void operator >> (BHumanMessage& m) const override;
  void operator << (const BHumanMessage& m) override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override,

  (char)(0)         confidenceOfLastWhistleDetection, /**< Confidence based on hearing capability */
  (unsigned int)(0) lastTimeWhistleDetected,          /**< Timestamp */
  (unsigned int)(0) lastTimeOfIncomingSound,          /**< The last point of time when the robot received audio data */
  (std::string)("") whistleName,                      /**< Name of the last detected whistle */
});

inline void Whistle::operator >> (BHumanMessage& m) const
{
  m.theBHULKsStandardMessage.lastTimeWhistleDetected = lastTimeWhistleDetected;
  m.theBHULKsStandardMessage.confidenceOfLastWhistleDetection = B_HULKs::HearingConfidence(confidenceOfLastWhistleDetection < 0 ? static_cast<char>(-1) : confidenceOfLastWhistleDetection);

  m.theBHumanArbitraryMessage.queue.out.bin << lastTimeOfIncomingSound;
  m.theBHumanArbitraryMessage.queue.out.bin << whistleName;
  m.theBHumanArbitraryMessage.queue.out.finishMessage(id());
}

inline void Whistle::operator << (const BHumanMessage& m)
{
  lastTimeWhistleDetected = m.toLocalTimestamp(m.theBHULKsStandardMessage.lastTimeWhistleDetected);
  confidenceOfLastWhistleDetection = static_cast<char>(m.theBHULKsStandardMessage.confidenceOfLastWhistleDetection);
  lastTimeOfIncomingSound = Time::getCurrentSystemTime(); // does not matter anyway...
}

inline bool Whistle::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());
  m.bin >> lastTimeOfIncomingSound;
  lastTimeOfIncomingSound = toLocalTimestamp(lastTimeOfIncomingSound);
  m.bin >> whistleName;
  return true;
}
