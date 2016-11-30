/**
 * @file NoWirelessSound.h
 *
 * Identified sounds for technical challenge
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(NoWirelessSound,
{,
  (unsigned int)(0) lastTimeSoundDetected,          /**< Timestamp */
  (unsigned int)(0) lastTimeOfIncomingSound,        /**< The last point of time when the robot received audio data */
  (int)(0)          soundNumber,                    /**< Number of the last detected sound (equals bit value) */
  (int)(-1)         numOfDifferentSounds,           /**< As the name says */
});
