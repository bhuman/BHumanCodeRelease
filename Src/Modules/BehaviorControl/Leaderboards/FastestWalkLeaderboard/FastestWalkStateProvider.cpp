/**
 * @file FastestWalkStateProvider.cpp
 *
 * @author Moritz Oppermann
 */

#include "FastestWalkStateProvider.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(FastestWalkStateProvider);

void FastestWalkStateProvider::update(FastestWalkState& theFastestWalkState)
{
  if(theGameState.isPenalized())
  {
    ASSERT(FastestWalkState::numOfWalkStates >= 1);
    const bool thisBumperState = theKeyStates.pressed[KeyStates::lFootLeft] || theKeyStates.pressed[KeyStates::lFootRight];
    if(lastBumperState && !thisBumperState && theFrameInfo.getTimeSince(lastSwitch) > 800)
    {
      walkStateIndex = (walkStateIndex + 1) % FastestWalkState::numOfWalkStates;
      state = FastestWalkState::WalkState(walkStateIndex);
      switch(state)
      {
      case FastestWalkState::WalkState::forward:
        SystemCall::say("Forward");
        break;
      case FastestWalkState::WalkState::slalom:
        SystemCall::say("Slalom");
        break;
      }
      lastSwitch = theFrameInfo.time;
    }
    lastBumperState = thisBumperState;
  }
  theFastestWalkState.state = state;
}
