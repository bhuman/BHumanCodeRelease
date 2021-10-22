/**
 * @file HeadOrientation.h
 *
 * @author Lukas Plecher
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(HeadOrientation,
{,
  (Angle)(0) pan,
  (Angle)(0) tilt,
});
