/**
 * @file AudioData.h
 * The file declares a struct that stores audio data of up to four channels.
 * On a V4, the four channels are:
 * 0: left microphone
 * 1: right microphone
 * 2: front microphone
 * 3: rear microphone
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(AudioData,
{,
  (unsigned)(2) channels,
  (unsigned)(48000) sampleRate,
  (std::vector<short>) samples, /**< Samples are interleaved. */
});
