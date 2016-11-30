#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(SignalStateWriterThread,
{,
  (bool) running,
});