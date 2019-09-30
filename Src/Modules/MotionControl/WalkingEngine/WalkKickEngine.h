/**
 * @file WalkKickEngine.h
 *
 * This file declares a module that provides a walk kick generator.
 *
 * @author Alexis Tsogias
 * @author Arne Hasselbring
 */

#pragma once

#include "Platform/SystemCall.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Tools/Math/CubicSpline.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include <array>


STREAMABLE(WalkKick,
{
  STREAMABLE(KeyFrame,
  {
    void onRead(),

    (float) phase, // The point in the phase where this keyframe is located, in range of (0 ... 1) (zero and one are exclusive!
    (Vector3f) position, // The target position.
    (Vector3a) rotation, // The target rotation as angle axis.
  });

  void onRead(),

  (Legs::Leg)(Legs::left) kickLeg, // The leg the kick is defined for.
  (bool)(false) requiresPreStep, // If a pre-step is required for the kick.
  (Pose2f) preStepSize, // The step size of the previous step if one is necessary.
  (Pose2f) stepSize, // The targeted step size.
  (std::vector<KeyFrame>) keyFrames, // The keyFrames.
});

inline void WalkKick::KeyFrame::onRead()
{
  ASSERT(phase > 0.f && phase < 1.f);
}

inline void WalkKick::onRead()
{
#ifndef NDEBUG
  if(keyFrames.size() > 0)
  {
    auto iter = keyFrames.begin();
    auto nextIter = iter + 1;
    auto end = keyFrames.end();
    while(nextIter < end)
    {
      ASSERT(iter->phase < nextIter->phase);
      nextIter = ++iter + 1;
    }
  }
#endif
}

MODULE(WalkKickEngine,
{,
  PROVIDES(WalkKickGenerator),
  DEFINES_PARAMETERS(
  {,
    (bool)(true) playKickSounds, /**< Say which kick is currently executed. */
  }),
});

class WalkKickEngine : public WalkKickEngineBase
{
public:
  WalkKickEngine();

  void start(WalkKickGenerator& walkKickGenerator, const WalkKickVariant& walkKickVariant);
  void getState(float phase, Vector3f& position, Vector3f& rotation);

private:

  void update(WalkKickGenerator& walkKickGenerator) override;

  void readWalkKicks();

  std::array<CubicSpline, 3> positionSplines;
  std::array<CubicSpline, 3> rotationSplines;

  std::array<std::string, WalkKicks::numOfTypes> kickTypesSpeech; // Stores the enums converted properly so the TTS can synthesize it

  ENUM_INDEXED_ARRAY(WalkKick, WalkKicks::Type) walkKicks;
};
