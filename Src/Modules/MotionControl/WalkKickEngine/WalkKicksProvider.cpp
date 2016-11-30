#include "WalkKicksProvider.h"
#include "Tools/Debugging/DebugRequest.h"

MAKE_MODULE(WalkKicksProvider, motionControl)

void WalkKicksProvider::update(WalkKicks& walkKicks)
{
  DEBUG_RESPONSE_ONCE("module:WalkKicksProvider:reloadKicks")
    loaded = false;

  if(!loaded)
    loadKicks(walkKicks);
}

void WalkKicksProvider::loadKicks(WalkKicks& walkKicks)
{
  for(int i = 1; i < WalkKicks::Type::numOfTypes; ++i)
  {
    const std::string kickName = WalkKicks::getName(static_cast<WalkKicks::Type>(i));

    InMapFile stream("WalkKicks/" + kickName + ".cfg");
    ASSERT(stream.exists());
    stream >> walkKicks.kicks[i];
  }
  loaded = true;
}