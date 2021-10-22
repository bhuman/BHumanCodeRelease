/**
 * @file EnergySaving.h
 * Current offsets, to reduce the current in the joints as much as possible
 * @author Philip Reichenberg
 */

#pragma once
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Function.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/EnumIndexedArray.h"

STREAMABLE(EnergySaving,
{
  ENUM(EnergyState,
  {,
    waiting,
    working,
    resetState,
  });
  EnergySaving();

  FUNCTION(void(JointRequest& request, const bool adjustLegs, const bool adjustArms, const bool standHigh, const bool accuratePositions)) applyHeatAdjustment;
  FUNCTION(void()) reset;
  FUNCTION(void()) shutDown,

  (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) offsets,
  (EnergyState)(EnergyState::waiting) state,
});

inline EnergySaving::EnergySaving()
{
  offsets.fill(0_deg);
}
