/**
 * @file LibDemoProvider.h
 *
 * Manages useful things for demos.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "Framework/Module.h"
#include "Debugging/DebugDrawings.h"
#include "Representations/BehaviorControl/Libraries/LibDemo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/RobotHealth.h"

MODULE(LibDemoProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(KeyStates),
  USES(RobotHealth),
  PROVIDES(LibDemo),
  LOADS_PARAMETERS(
  {,
    (bool) isDemoActive,
    (bool) isOneVsOneDemoActive,
    (bool) isMiniFieldActive,
    (LibDemo::StdVectorDemoGameState) activeDemoGameStates,
    (bool) changeArmToWave,
  }),
});

class LibDemoProvider : public LibDemoProviderBase
{
  void update(LibDemo& libDemo) override;

  /**
   * In soccer mode, replace soccer with waving when in heat
   */
  void handleHeatInSoccer();

  LibDemo::DemoGameState demoGameState = LibDemo::DemoGameState::soccer;
  unsigned lastSwitch = 0;
  bool lastBumperState = false;
  unsigned demoGameStateIndex = 0;
};
