/**
 * @file AgentStates.h
 *
 * This file declares a representation of the internal state of the strategy.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"
#include <vector>

STREAMABLE(AgentStates,
{
  void draw() const,

  (std::vector<Agent>) agents, /**< The states of the team's agents as modeled by the behavior. */
});
