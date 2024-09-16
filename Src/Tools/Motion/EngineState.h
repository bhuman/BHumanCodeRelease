/**
 * @file EngineState.h
 * Contains the List of all available states of the KeyframeMotionEngine
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/Enum.h"

namespace EngineStates
{
  ENUM(EngineState,
  {, //all States defined in getUpEngine.cpp
    off,
    helpMeState,
    breakUp,
    decideAction,
    waiting,
    waitForRequest,
    working,
    finished,
    balanceOut,
    retryState,
  });

  using EngineStateVector = std::vector<EngineState>;
}
