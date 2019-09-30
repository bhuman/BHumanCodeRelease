/**
 * @file GetUpMotionPhase.h
 * Declaration of a struct for representing the phases for get up motions
 * @author <a href="mailto:s_ksfo6n@uni-bremen.de">Philip Reichenberg</a>
 */

#pragma once
#include "Tools/Streams/Enum.h"

namespace GetUpMotionPhases
{
  ENUM(GetUpMotionPhase,
  {,
    ErrorPhase,
    Lying,
    ArmPushing,
    BackSitUp,
    PushingWithArmsFromGround,
    ReduceVel,
    Splitt,
    HalfSplitt,
    Sit,
    HighSitVel,
    Stand,
  });
  using GetUpMotionPhaseVector = std::vector<GetUpMotionPhase>;
}
