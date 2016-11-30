#pragma once

#include "Tools/Streams/AutoStreamable.h"

enum class Direction { front, back, left, right };

STREAMABLE(TopplingOver,
{,
  (bool) on_ground,
  (bool) toppling,
  (bool) (false) front,
  (bool) (false) back,
  (bool) (false) left,
  (bool) (false) right,
});