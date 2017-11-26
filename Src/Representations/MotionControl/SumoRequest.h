/**
 * @file Representations/MotionControl/kickRequest.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(SumoRequest,
{
  ENUM(SumoMotionID,
  {,
    Sumo,
  });

  static SumoMotionID getSumoMotionFromName(const char* name),

  (SumoMotionID)(Sumo) SumoMotionType,
  (bool)(false) mirror,
  (bool)(false) armsBackFix,
  (std::vector<DynPoint>) dynPoints,
});
