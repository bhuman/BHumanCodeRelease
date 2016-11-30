/**
 * @file Whistle.h
 *
 * Identified whistle sound
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(Whistle,
{,
  (char)(0)         confidenceOfLastWhistleDetection, /**< Confidence based on hearing capability */
  (unsigned int)(0) lastTimeWhistleDetected,          /**< Timestamp */
  (unsigned int)(0) lastTimeOfIncomingSound,          /**< The last point of time when the robot received audio data */
  (std::string)("") whistleName,                      /**< Name of the last detected whistle */
});
