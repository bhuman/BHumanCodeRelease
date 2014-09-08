/**
* @file TeammateReliability.h
*
* Declaration of class TeammateReliability.
* 
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Representations/Infrastructure/TeammateData.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

/**
* @class SideConfidence
*/
STREAMABLE(TeammateReliability,
{
public:
  ENUM(ReliabilityState,
    UNKNOWN,
    CHECKING,
    UNRELIABLE,
    OK,
    GOOD
  );                                                     /**< Discrete states of reliability */

  TeammateReliability()
  {
    for(int i = 0; i<TeammateData::numOfPlayers; i++)
      states[i] = UNKNOWN;
  }

  void draw(),
  
  (ReliabilityState[TeammateData::numOfPlayers]) states, /**< The states of my teammates */
});
