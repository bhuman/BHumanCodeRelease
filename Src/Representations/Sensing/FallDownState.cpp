/**
 * @file Representations/Sensing/FallDownState.cpp
 *
 * Implementation of a debug drawing for the FallDownState.
 *
 * @author Carsten Könemann
 */

#include "FallDownState.h"
#include "Tools/Debugging/DebugDrawings.h"

void FallDownState::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:FallDownState", "drawingOnImage");
  // text-shadow for better visibility
  DRAWTEXT("representation:FallDownState", 26, 26, 35, ColorClasses::black, "State: " << getName(state));
  DRAWTEXT("representation:FallDownState", 26, 51, 35, ColorClasses::black, "Direction: " << getName(direction));
  DRAWTEXT("representation:FallDownState", 26, 76, 35, ColorClasses::black, "Sidewards: " << getName(sidewards));
  // text
  DRAWTEXT("representation:FallDownState", 25, 25, 35, ColorClasses::white, "State: " << getName(state));
  DRAWTEXT("representation:FallDownState", 25, 50, 35, ColorClasses::white, "Direction: " << getName(direction));
  DRAWTEXT("representation:FallDownState", 25, 75, 35, ColorClasses::white, "Sidewards: " << getName(sidewards));
}
