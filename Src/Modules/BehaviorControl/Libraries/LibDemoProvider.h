/**
 * @file LibDemoProvider.h
 *
 * Manages usefull things for demos.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "Tools/Module/Module.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/BehaviorControl/Libraries/LibDemo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

MODULE(LibDemoProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(KeyStates),
  REQUIRES(RobotInfo),
  PROVIDES(LibDemo),
  LOADS_PARAMETERS(
  {,
    (bool) isDemoActive,
    ((LibDemo) StdVectorDemoGameState) activeDemoGameStates,
    (bool) changeArmToWave,
  }),
});

class LibDemoProvider : public LibDemoProviderBase
{
  void update(LibDemo& libDemo);

  LibDemo::DemoGameState demoGameState = LibDemo::DemoGameState::normal;
  unsigned lastSwitch = 0;
  bool lastBumperState = false;
  Arms::Arm armToWave = Arms::left;
  unsigned demoGameStateIndex = 0;
};
