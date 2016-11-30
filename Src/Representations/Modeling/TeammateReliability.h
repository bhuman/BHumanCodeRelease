/**
 * @file TeammateReliability.h
 *
 * Declaration of struct TeammateReliability.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"

STREAMABLE(TeammateReliability,
{
  ENUM(ReliabilityState,
  {,
    UNKNOWN,
    CHECKING,
    UNRELIABLE,
    OK,
    GOOD,
  });                                                     /**< Discrete states of reliability */

  TeammateReliability()
  {
    states.resize(Settings::highestValidPlayerNumber + 1);
    for(unsigned int i = 0; i < states.size(); i++)
      states[i] = UNKNOWN;
  }

  void draw() const,

  (std::vector<ReliabilityState>) states, /**< The states of my teammates */
});
