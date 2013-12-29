#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(FsrData,
{,
  (bool)(true) leftFootContact,
  (bool)(true) rightFootContact,
  (Vector2<>) centerOfPressure,
});
