/**
 * @file EnergySaving.h
 * Current offsets, to reduce the current in the joints as much as possible
 * @author Philip Reichenberg
 */

#pragma once
#include "Representations/Infrastructure/JointRequest.h"
#include "Streaming/Function.h"
#include "Streaming/Enum.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(EnergySaving,
{
  ENUM(EnergyState,
  {,
    off,
    waiting,
    working,
    resetState,
  });
  EnergySaving();

  FUNCTION(void(JointRequest& request, bool adjustLeftLeg, bool adjustRightLeg, bool adjustLeftArm, bool adjustRightArm, const bool standHigh, const bool onlyBaseOffset)) applyHeatAdjustment;
  FUNCTION(void()) reset;
  FUNCTION(void()) shutDown,

  (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) offsets,
  (EnergyState)(EnergyState::off) state,
});

inline EnergySaving::EnergySaving()
{
  offsets.fill(0_deg);
}
