/**
 * @file Representations/MotionControl/kickRequest.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(GroundPunchRightRequest,
{
  ENUM(GroundPunchRightMotionID,
  {,
    GroundPunchRight,
  });

  static GroundPunchRightMotionID getGroundPunchRightMotionFromName(const char* name),

  (GroundPunchRightMotionID)(GroundPunchRight) GroundPunchRightMotionType,
  (bool)(false) mirror,
  (bool)(false) armsBackFix,
  (std::vector<DynPoint>) dynPoints,
});
