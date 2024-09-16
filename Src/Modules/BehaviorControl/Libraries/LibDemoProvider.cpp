/**
 * @file LibDemoProvider.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "Platform/SystemCall.h"
#include "LibDemoProvider.h"

MAKE_MODULE(LibDemoProvider);

void LibDemoProvider::update(LibDemo& libDemo)
{
  if(isDemoActive && theGameState.isPenalized())
  {
    ASSERT(activeDemoGameStates.size() >= 1);
    const bool thisBumperState = theKeyStates.pressed[KeyStates::lFootLeft] || theKeyStates.pressed[KeyStates::lFootRight];
    if(activeDemoGameStates.size() >= 1 &&
       lastBumperState && !thisBumperState && theFrameInfo.getTimeSince(lastSwitch) > 800)
    {
      demoGameStateIndex = (demoGameStateIndex + 1) % activeDemoGameStates.size();
      demoGameState = activeDemoGameStates[demoGameStateIndex];
      switch(demoGameState)
      {
        case LibDemo::DemoGameState::waving:
          SystemCall::say("Waving");
          break;
        case LibDemo::DemoGameState::talking:
          SystemCall::say("Talking");
          break;
        case LibDemo::DemoGameState::soccer:
          SystemCall::say("Soccer");
          break;
        case LibDemo::DemoGameState::posing:
          SystemCall::say("Posing");
          break;
      }
      lastSwitch = theFrameInfo.time;
    }

    lastBumperState = thisBumperState;
  }
  handleHeatInSoccer();
  libDemo.isDemoActive = isDemoActive;
  libDemo.demoGameState = demoGameState;
  libDemo.changeArmToWave = changeArmToWave;
  libDemo.isOneVsOneDemoActive = isOneVsOneDemoActive;
  libDemo.isMiniFieldActive = isMiniFieldActive;
}

void LibDemoProvider::handleHeatInSoccer()
{
  if(theRobotHealth.maxJointTemperatureStatus != JointSensorData::TemperatureStatus::regular &&
     demoGameState == LibDemo::DemoGameState::soccer)
    demoGameState = LibDemo::DemoGameState::waving;
}
