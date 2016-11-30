/**
 * Status of the NoWirelessDataSender.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(NoWirelessDataSenderStatus,
{,
  (bool)(false) connected,
});
