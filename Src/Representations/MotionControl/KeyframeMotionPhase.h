/**
 * @file KeyframeMotionPhase.h
 * Declaration of a struct for representing the phases for keyframe based motions
 * @author <a href="mailto:s_ksfo6n@uni-bremen.de">Philip Reichenberg</a>
 */

#pragma once
#include "Tools/Streams/Enum.h"

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
    HighSitVel,
    Stand,
  });
  using KeyframeMotionPhaseVector = std::vector<KeyframeMotionPhase>;
}
