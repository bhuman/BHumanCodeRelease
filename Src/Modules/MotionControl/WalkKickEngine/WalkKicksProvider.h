#pragma once

#include "Representations/MotionControl/WalkKicks.h"
#include "Tools/Module/Module.h"

MODULE(WalkKicksProvider,
{,
  PROVIDES(WalkKicks),
});

class WalkKicksProvider : public WalkKicksProviderBase
{
  bool loaded = false;

  void update(WalkKicks& walkKicks);

  void loadKicks(WalkKicks& walkKicks);
};