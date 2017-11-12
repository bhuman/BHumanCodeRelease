#pragma once

#include "Representations/Configuration/WalkKicks.h"
#include "Tools/Math/CubicSpline.h"
#include <array>

class WalkKickEngine
{
public:
  WalkKickEngine(const WalkKicks& walkKicks);

  void start(WalkKicks::Type kickType, Legs::Leg kickLeg);
  void getState(float phase, Vector3f& position, Vector3f& rotation);
  WalkKicks::Type getCurrentKickType() const { return currentKickType; }
  Legs::Leg getCurrentKickLeg() const { return currentKickLeg; }

private:
  const WalkKicks& walkKicks;

  WalkKicks::Type currentKickType = WalkKicks::none;
  Legs::Leg currentKickLeg = Legs::left;

  std::array<CubicSpline, 3> positionSplines;
  std::array<CubicSpline, 3> rotationSplines;
};
