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
#include "Tools/Math/Angle.h"

#include <algorithm>
#include <limits>

STREAMABLE(Whistle, COMMA public BHumanMessageParticle<idWhistle>
{
  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override,

  (float)(0)         confidenceOfLastWhistleDetection, /**< Confidence based on hearing capability. */
  (unsigned char)(0) channelsUsedForWhistleDetection,  /**< Number of channels the robot used to listen. */
  (unsigned int)(0)  lastTimeWhistleDetected,          /**< Timestamp */
});

inline void Whistle::operator>>(BHumanMessage& m) const
{
  m.theBHumanStandardMessage.lastTimeWhistleDetected = lastTimeWhistleDetected;
  m.theBHumanStandardMessage.confidenceOfLastWhistleDetection = static_cast<unsigned char>(std::min(confidenceOfLastWhistleDetection * 100.f, 255.f));
  m.theBHumanStandardMessage.channelsUsedForWhistleDetection = channelsUsedForWhistleDetection;
}

inline void Whistle::operator<<(const BHumanMessage& m)
{
  if(m.hasBHumanParts)
  {
    lastTimeWhistleDetected = m.toLocalTimestamp(m.theBHumanStandardMessage.lastTimeWhistleDetected);
    confidenceOfLastWhistleDetection = m.theBHumanStandardMessage.confidenceOfLastWhistleDetection / 100.f;
    channelsUsedForWhistleDetection = m.theBHumanStandardMessage.channelsUsedForWhistleDetection;
  }
  else
    channelsUsedForWhistleDetection = 0;
}
