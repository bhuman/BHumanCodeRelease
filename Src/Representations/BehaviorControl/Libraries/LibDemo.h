/**
 * @file LibDemo.h
 *
 * This file defines a representation that contains methods that calculate cool demo stuff
 *
 * @author Daniel Krause
 */

#pragma once
#include "Streaming/Function.h"
#include "RobotParts/Arms.h"
#include "Streaming/Enum.h"
#include "Representations/MotionControl/KickRequest.h"

STREAMABLE(LibDemo,
{
  ENUM(DemoGameState,
  {,
    soccer,
    waving,
    talking,
    posing,
  });

  using StdVectorDemoGameState = std::vector<LibDemo::DemoGameState>,

  (bool) isDemoActive,
  (bool) changeArmToWave,
  (bool) isOneVsOneDemoActive,
  (bool) isMiniFieldActive,
  (LibDemo::DemoGameState) demoGameState,
});
