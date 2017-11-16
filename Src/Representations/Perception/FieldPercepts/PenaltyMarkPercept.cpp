/**
 * @file PenaltyMarkPercept.cpp
 * Implementation of a struct that represents a penalty mark.
 * @author Maik Schünemann
 */

#include "PenaltyMarkPercept.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"

void PenaltyMarkPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:PenaltyMarkPercept:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:PenaltyMarkPercept:field", "drawingOnField");

  if(wasSeen)
  {
    CROSS("representation:PenaltyMarkPercept:image", positionInImage.x(), positionInImage.y(), 5, 5, Drawings::solidPen, ColorRGBA::blue);
    CROSS("representation:PenaltyMarkPercept:field", positionOnField.x(), positionOnField.y(), 40, 40, Drawings::solidPen, ColorRGBA::blue);
  }
}
