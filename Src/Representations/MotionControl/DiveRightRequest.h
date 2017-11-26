/**
 * @file Representations/MotionControl/kickRequest.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(DiveRightRequest,
{
  ENUM(DiveRightMotionID,
  {,
    DiveRight,
  });

  static DiveRightMotionID getDiveRightMotionFromName(const char* name),

  (DiveRightMotionID)(DiveRight) DiveRightMotionType,
  (bool)(false) mirror,
  (bool)(false) armsBackFix,
  (std::vector<DynPoint>) dynPoints,
});
