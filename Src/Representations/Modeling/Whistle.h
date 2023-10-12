/**
 * @file Whistle.h
 *
 * Identified whistle sound
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Communication/BHumanMessageParticle.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(Whistle, COMMA public BHumanCompressedMessageParticle<Whistle>
{
  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override,

  (float)(0)         confidenceOfLastWhistleDetection, /**< Confidence based on hearing capability. */
  (unsigned char)(0) channelsUsedForWhistleDetection,  /**< Number of channels the robot used to listen. */
  (unsigned int)(0)  lastTimeWhistleDetected,          /**< Timestamp */
});

STREAMABLE(RecentWhistle,
{
  RecentWhistle() = default;
  RecentWhistle(float confidenceOfLastWhistleDetection, unsigned int lastTimeWhistleDetected)
    : confidenceOfLastWhistleDetection(confidenceOfLastWhistleDetection) COMMA
      lastTimeWhistleDetected(lastTimeWhistleDetected) {},

  (float)(0)        confidenceOfLastWhistleDetection, /**< Confidence based on hearing capability. */
  (unsigned int)(0) lastTimeWhistleDetected,          /**< Timestamp */
});

STREAMABLE(WhistleCompact,
{
private:
  Whistle* whistle = nullptr;

public:
  WhistleCompact(const Whistle& whistle);
  WhistleCompact(Whistle& whistle) : whistle(&whistle) {}
  void onRead(),

  (bool)(true) listening, /**< Whether the robot is listening (0 or 2 channels). */
  (std::optional<RecentWhistle>) recentWhistle, /**< The whistle if it is not too old. */
});
