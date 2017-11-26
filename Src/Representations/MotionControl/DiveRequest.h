/**
 * @file Representations/MotionControl/kickRequest.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(DiveRequest,
{
  ENUM(DiveMotionID,
  {,
    Dive,
  });

  static DiveMotionID getDiveMotionFromName(const char* name),

  (DiveMotionID)(Dive) DiveMotionType,
  (bool)(false) mirror,
  (bool)(false) armsBackFix,
  (std::vector<DynPoint>) dynPoints,
});
