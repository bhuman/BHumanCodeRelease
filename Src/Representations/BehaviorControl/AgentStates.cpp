/**
 * @file AgentStates.cpp
 *
 * This file implements a draw method for the agent states.
 *
 * @author Arne Hasselbring
 */

#include "AgentStates.h"
#include "Debugging/DebugDrawings.h"
#include <string>

void AgentStates::draw() const
{
  DEBUG_DRAWING("representation:AgentStates", "drawingOnField")
  {
    for(const Agent& agent : agents)
    {
      DRAW_TEXT("representation:AgentStates", agent.lastKnownPose.translation.x() - 50, agent.lastKnownPose.translation.y() - 50, 100, ColorRGBA::black, std::to_string(agent.number));
      CIRCLE("representation:AgentStates", agent.lastKnownPose.translation.x(), agent.lastKnownPose.translation.y(), 100, 10, Drawings::dashedPen, ColorRGBA::black, Drawings::noBrush, ColorRGBA::black);
      LINE("representation:AgentStates", agent.lastKnownPose.translation.x(), agent.lastKnownPose.translation.y(), agent.currentPosition.x(), agent.currentPosition.y(), 10, Drawings::dashedPen, ColorRGBA::black);
      CROSS("representation:AgentStates", agent.currentPosition.x(), agent.currentPosition.y(), 100, 10, Drawings::dashedPen, ColorRGBA::black);
    }
  }
}
