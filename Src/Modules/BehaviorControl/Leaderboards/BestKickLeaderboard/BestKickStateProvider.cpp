/**
 * @file BestKickStateProvider.cpp
 *
 * @author Moritz Oppermann
 */

#include "BestKickStateProvider.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(BestKickStateProvider);

void BestKickStateProvider::update(BestKickState& theBestKickState)
{
  if(theGameState.isPenalized())
  {
    ASSERT(BestKickState::numOfTargets >= 1);
    const bool thisBumperState = theKeyStates.pressed[KeyStates::lFootLeft] || theKeyStates.pressed[KeyStates::lFootRight];
    if(lastBumperState && !thisBumperState && theFrameInfo.getTimeSince(lastSwitch) > 800)
    {
      targetIndex = (targetIndex + 1) % BestKickState::numOfTargets;
      target = BestKickState::Target(targetIndex);
      switch(target)
      {
      case BestKickState::Target::a:
        SystemCall::say("A");
        break;
      case BestKickState::Target::b:
        SystemCall::say("B");
        break;
      case BestKickState::Target::c:
        SystemCall::say("C");
        break;
      }
      lastSwitch = theFrameInfo.time;
    }
    lastBumperState = thisBumperState;
  }
  theBestKickState.target = target;
}
