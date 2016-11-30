/**
 * @file Representations/MotionControl/kickRequest.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(KickRequest,
{
  ENUM(KickMotionID,
  {,
    kickForward,
    newKick,
    none,
  });

  static KickMotionID getKickMotionFromName(const char* name),

  (KickMotionID)(none) kickMotionType,
  (bool)(false) mirror,
  (bool)(false) armsBackFix,
  (std::vector<DynPoint>) dynPoints,
});
