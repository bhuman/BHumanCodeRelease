/**
 * @file GetUpPhase.h
 * Declaration of a struct for representing the phases for get up motions
 * @author <a href="mailto:s_ksfo6n@uni-bremen.de">Philip Reichenberg</a>
 */

#pragma once
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include "Representations/MotionControl/GetUpMotionPhase.h"

STREAMABLE(GetUpPhase,
{
  STREAMABLE(BalanceFactors,
  {,
    (Vector2f)(0.f, 0.f) increaseFactorCapFront,
    (Vector2f)(0.f, 0.f) increaseFactorCapBack,
    (Vector2f)(0.f, 0.f) powDegreeFront,
    (Vector2f)(0.f, 0.f) powDegreeBack,
    (Vector2f)(0.f, 0.f) borderPIDPX,
    (Vector2f)(0.f, 0.f) borderPIDPY,
    (Vector2f)(0.f, 0.f) borderPIDDX,
    (Vector2f)(0.f, 0.f) borderPIDDY,
  });

  STREAMABLE(BalancingParameter2,
  {,
    (BalanceFactors) balanceFactor,
    (Vector2f)(-2000.f, -2000.f) backwardCOMDif,
    (Vector2a)(-2000_deg, -2000_deg) backwardAngleBreakUp,
    (Vector2f)(2000.f, 2000.f) forwardCOMDif,
    (Vector2a)(2000_deg, 2000_deg) forwardAngleBreakUp,
  }),

  (ENUM_INDEXED_ARRAY(BalancingParameter2, GetUpMotionPhases::GetUpMotionPhase)) balanceList,
});
