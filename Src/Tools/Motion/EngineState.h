/**
 * @file EngineState.h
 * Contains the List of all available states of the getUpEngine.h
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Tools/Streams/Enum.h"

namespace EngineStates
{
  ENUM(EngineState,
  {, //all States defined in getUpEngine.cpp
    off,
    helpMeState,
    pickUp,
    breakUp,
    waiting,
    decideAction,
    recoverFallen,
    working,
    finished,
    finishedRecover,
    balanceOut,
    retryState,
  });

  using EngineStateVector = std::vector<EngineState>;
}
