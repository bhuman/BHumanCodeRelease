/**
 * @file LibDemoProvider.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "Platform/SystemCall.h"
#include "LibDemoProvider.h"

MAKE_MODULE(LibDemoProvider, behaviorControl);

void LibDemoProvider::update(LibDemo& libDemo)
{
  if(isDemoActive && theRobotInfo.penalty != PENALTY_NONE)
  {
    ASSERT(activeDemoGameStates.size() >= 1);
    const bool thisBumperState = theKeyStates.pressed[KeyStates::leftFootLeft] || theKeyStates.pressed[KeyStates::leftFootRight];
    if(activeDemoGameStates.size() >= 1 &&
       lastBumperState && !thisBumperState && theFrameInfo.getTimeSince(lastSwitch) > 800)
    {
      demoGameStateIndex = (demoGameStateIndex + 1) % activeDemoGameStates.size();
      demoGameState = activeDemoGameStates[demoGameStateIndex];
      switch(demoGameState)
      {
        case LibDemo::DemoGameState::waving:
          SystemCall::playSound("greetings.wav");
          break;
        case LibDemo::DemoGameState::normal:
          SystemCall::playSound("normalPlay.wav");
          break;
      }
      lastSwitch = theFrameInfo.time;
    }

    lastBumperState = thisBumperState;
  }
  libDemo.isDemoActive = isDemoActive;
  libDemo.armToWave = armToWave;
  libDemo.demoGameState = demoGameState;
  libDemo.changeArmToWave = changeArmToWave;
  libDemo.setArmToWave = [this, &libDemo](Arms::Arm arm)
  {
    libDemo.armToWave = arm;
    armToWave = arm;
  };
}
