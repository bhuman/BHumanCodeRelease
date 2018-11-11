/**
 * @file CognitionStateChanges.h
 * The file declares a representation, which holds the state of the last cognition frame from various states.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(CognitionStateChanges,
{,
  (int)(0) lastGameState,
  (int)(0) lastGamePhase,
  (int)(0) lastPenalty,
  (int)(0) lastSetPlay,
});
