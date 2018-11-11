#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Streams/EnumIndexedArray.h"

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
  (bool)(false) requiresPrestep, // If a prestep is required for the kick.
  (Pose2f) preStepSize, // The stepsize of the previous step if one is necessary.
  (float)(0.f) duration, // The duration of the kick in s.
  (Pose2f) stepSize, // The Targeted stepsize.
  (Vector2f)(Vector2f::Zero()) origin, // The base of the pendulum during the kick relative to the origin of the support foot.
  (std::vector<KeyFrame>) keyFrames, // The keyFrames.
});

STREAMABLE(WalkKicks,
{
  ENUM(Type,
  {,
    none,
    forward,
  }),

  (ENUM_INDEXED_ARRAY(WalkKick, Type)) kicks,
});

STREAMABLE(WalkKickVariant,
{
  WalkKickVariant() = default;
  WalkKickVariant(WalkKicks::Type kickType, Legs::Leg kickLeg);

  bool operator==(const WalkKickVariant& other) const,

  (WalkKicks::Type)(WalkKicks::none) kickType,
  (Legs::Leg)(Legs::left) kickLeg,
});

inline WalkKickVariant::WalkKickVariant(WalkKicks::Type kickType, Legs::Leg kickLeg) : kickType(kickType), kickLeg(kickLeg) {}

inline bool WalkKickVariant::operator==(const WalkKickVariant& other) const
{
  return kickType == other.kickType && kickLeg == other.kickLeg;
}

inline void WalkKick::KeyFrame::onRead()
{
  ASSERT(phase > 0.f && phase < 1.f);
}

inline void WalkKick::onRead()
{
  if(keyFrames.size() > 0)
  {
    ASSERT(duration > 0.f);
    auto iter = keyFrames.begin();
    auto nextIter = iter + 1;
    auto end = keyFrames.end();
    while(nextIter < end)
    {
      ASSERT(iter->phase < nextIter->phase);
      nextIter = ++iter + 1;
    }
  }
}
