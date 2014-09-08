/**
* @file ReceivedSplStandardMessages.h
* Contains the recently received SPLStandardMessage of each teammate
* to be used in DropInChallenge
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#pragma once

#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/SPLStandardMessage.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(ReceivedSPLStandardMessages,
{
public:
  ReceivedSPLStandardMessages()
  {
    memset(timestamps, 0, sizeof(timestamps));
  },

  (SPLStandardMessageWithoutData[TeammateData::numOfPlayers]) messages,
  (unsigned[TeammateData::numOfPlayers]) timestamps,
});
