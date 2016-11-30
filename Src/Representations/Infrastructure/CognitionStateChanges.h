/**
 * @file CognitionStateChanges.h
 * The file declares a represtation, which holds the state of the last cognition frame from varios states.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(CognitionStateChanges,
{,
  (int)(0) lastGameState,
  (int)(0) lastPenalty,
});
