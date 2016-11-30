#pragma once

#include "Representations/MotionControl/WalkKicks.h"
#include "Tools/Math/CubicSpline.h"

class WalkKickEngine
{
private:
  const WalkKicks& walkKicks;

  WalkKicks::Type currentKickType = WalkKicks::none;
  Legs::Leg currentKickLeg = Legs::left;

  CubicSpline positionSplines[3];
  CubicSpline rotationSplines[3];

public:
  WalkKickEngine(const WalkKicks& walkKicks);

  void start(WalkKicks::Type kickType, Legs::Leg kickLeg);
  void getState(float phase, Vector3f& position, Vector3f& rotation);
  WalkKicks::Type getCurrentKickType() const { return currentKickType; }
  Legs::Leg getCurrentKickLeg() const { return currentKickLeg; }
};