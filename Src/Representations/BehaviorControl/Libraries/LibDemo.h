/**
 * @file LibDemo.h
 *
 * This file defines a representation that contains methods that calculate cool demo stuff
 *
 * @author Daniel Krause
 */

#pragma once
#include "Tools/Function.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/Enum.h"

STREAMABLE(LibDemo,
{
  ENUM(DemoGameState,
  {,
    waving,
    normal,
  });
  FUNCTION(void(Arms::Arm armToWave)) setArmToWave;

  using StdVectorDemoGameState = std::vector<LibDemo::DemoGameState>,
  
  ((Arms) Arm) armToWave,
  (bool) isDemoActive,
  (bool) changeArmToWave,
  ((LibDemo) DemoGameState) demoGameState,
});
