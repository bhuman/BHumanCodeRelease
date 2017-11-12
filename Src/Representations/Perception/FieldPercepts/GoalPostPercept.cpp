/**
 * @file GoalPostPercept.cpp
 * Implementation of a struct that represents a goal post.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "GoalPostPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void GoalPostPercept::draw() const
{
  static const ColorRGBA goalPostPerceptColor = ColorRGBA::red;

  DECLARE_DEBUG_DRAWING("representation:GoalPostPercept:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:GoalPostPercept:field", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:GoalPostPercept", "robot");
  TRANSLATE3D("representation:GoalPostPercept", 0, 0, -230);

  if(!wasSeen)
    return;

  CIRCLE("representation:GoalPostPercept:field", positionOnField.x(), positionOnField.y(),
         50, 0, Drawings::solidPen, ColorRGBA(255, 0, 0), Drawings::solidBrush, goalPostPerceptColor);
  MID_DOT("representation:GoalPostPercept:image", positionInImage.x(), positionInImage.y(),
          ColorRGBA(255, 0, 0), goalPostPerceptColor);
  LINE("representation:GoalPostPercept:image", positionInImage.x(), positionInImage.y(), positionInImage.x(), 0,
       5, Drawings::dottedPen, goalPostPerceptColor);

  CYLINDER3D("representation:GoalPostPercept", positionOnField.x(), positionOnField.y(), 400, 0, 0, 0, 50, 800, goalPostPerceptColor);
  LINE3D("representation:GoalPostPercept", 0, 0, 1.f, positionOnField.x(), positionOnField.y(), 1.f, 2.f, goalPostPerceptColor);
}
