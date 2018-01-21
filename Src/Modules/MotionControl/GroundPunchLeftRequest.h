/**
 * @file Representations/MotionControl/kickRequest.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(GroundPunchLeftRequest,
{
  ENUM(GroundPunchLeftMotionID,
  {,
    GroundPunchLeft,
  });

  static GroundPunchLeftMotionID getGroundPunchLeftMotionFromName(const char* name),

  (GroundPunchLeftMotionID)(GroundPunchLeft) GroundPunchLeftMotionType,
  (bool)(false) mirror,
  (bool)(false) armsBackFix,
  (std::vector<DynPoint>) dynPoints,
});
