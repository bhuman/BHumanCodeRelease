/**
 * @file ReceivedSplStandardMessages.h
 * Contains the recently received SPLStandardMessage of each teammate
 * to be used in DropInChallenge
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#pragma once

#include "Representations/Communication/SPLStandardMessage.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(ReceivedSPLStandardMessages,
{,
  (std::vector<SPLStandardMessageWithoutData>) messages,
  (std::vector<unsigned>) timestamps,
});
