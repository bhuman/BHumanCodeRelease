/**
 * @file KeyframeMotionPhase.h
 * Declaration of a struct for representing the phases for keyframe based motions
 * @author <a href="mailto:s_ksfo6n@uni-bremen.de">Philip Reichenberg</a>
 */

#pragma once
#include "Streaming/Enum.h"

namespace KeyframeMotionPhases
{
  ENUM(KeyframeMotionPhase,
  {,
    ErrorPhase,
    Lying,
    ArmPushing,
    BackSitUp,
    PushingWithArmsFromGround,
    ReduceVel,
    Split,
    HalfSplit,
    Sit,
    Stand,
    StandStatic,
  });
  using KeyframeMotionPhaseVector = std::vector<KeyframeMotionPhase>;
}
