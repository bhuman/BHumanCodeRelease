/*
 * @file SoundSignal.h
 *
 * Identified audio signals for the
 * Sound Recognition Challenge 2014
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(SoundSignal,
{,
  (unsigned int)(0) lastTimeAFSKSignalDetected,
  (unsigned int)(0) lastTimeWhistleDetected,
  (unsigned int)(0) lastTimeOfIncomingSound, /**< The last point of time when the robot heard something in general (must not be one of the signals */
});
