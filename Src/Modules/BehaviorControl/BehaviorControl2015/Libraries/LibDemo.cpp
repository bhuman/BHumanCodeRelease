/**
* @file LibDemo.cpp
*
* @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
*/

#include "../LibraryBase.h"
#include "Platform/SystemCall.h"

namespace Behavior2015
{
#include "LibDemo.h"

  LibDemo::LibDemo() : demoGameState(normal), lastSwtich(0), lastBumperState(false)
  {
    InMapFile stream("BehaviorControl2015/libDemo.cfg");
    ASSERT(stream.exists());
    stream >> parameters;
  }

  void LibDemo::preProcess()
  {
    MODIFY("parameters:BehaviorControl2015:libDemo", parameters);

    if(parameters.isDemoActive && theRobotInfo.penalty != PENALTY_NONE)
    {
      const bool thisBumperState = theKeyStates.pressed[KeyStates::leftFootLeft] || theKeyStates.pressed[KeyStates::leftFootRight];
      if(lastBumperState && !thisBumperState && theFrameInfo.getTimeSince(lastSwtich) > 800)
      {
        demoGameState = DemoGameState((demoGameState + 1) % numOfDemoGameStates);
        switch(demoGameState)
        {
        case wavingPlinking:
          SystemCall::playSound("greetings.wav");
          break;
        case normal:
          SystemCall::playSound("normalPlay.wav");
          break;
        default:
          break;
        }
        lastSwtich = theFrameInfo.time;
      }

      lastBumperState = thisBumperState;
    }
  }

}
