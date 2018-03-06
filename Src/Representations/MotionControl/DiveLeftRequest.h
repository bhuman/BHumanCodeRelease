/**
 * @file Representations/MotionControl/kickRequest.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(DiveLeftRequest,
{
  ENUM(DiveLeftMotionID,
  {,
    DiveLeft,
  });

  static DiveLeftMotionID getDiveLeftMotionFromName(const char* name),

  (DiveLeftMotionID)(DiveLeft) DiveLeftMotionType,
  (bool)(false) mirror,
  (bool)(false) armsBackFix,
  (std::vector<DynPoint>) dynPoints,
});
