/**
* @file Representations/MotionControl/BikeRequest.h
* @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
*/

#pragma once

#include "Modules/MotionControl/BIKEParameters.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(BikeRequest,
{
public:
  ENUM(BMotionID,
    kickForward,
    newKick,
    none
  );

  static BMotionID getBMotionFromName(const char* name),

  (BMotionID)(none) bMotionType,
  (bool)(false) mirror,
  (bool)(false) dynamical,
  (std::vector<DynPoint>) dynPoints,
});
