/**
 * @file GoalPercept.h
 *
 * Representation of a seen goal
 *
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#include "GoalPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void GoalPercept::draw() const
{
  const ColorRGBA goalPerceptColor = ColorRGBA::yellow;

  DECLARE_DEBUG_DRAWING("representation:GoalPercept:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:GoalPercept:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:GoalPercept", "robot");
  TRANSLATE3D("representation:GoalPercept", 0, 0, -230);
  ColorRGBA color = ColorRGBA(220, 220, 220);

  for(unsigned int i = 0; i < goalPosts.size(); ++i)
  {
    const GoalPost& p = goalPosts.at(i);
    if(p.position != GoalPost::IS_UNKNOWN)
    {
      CIRCLE("representation:GoalPercept:Field", p.positionOnField.x(), p.positionOnField.y(),
             50, 0, Drawings::solidPen, goalPerceptColor, Drawings::solidBrush, color);
      LINE("representation:GoalPercept:Field", p.positionOnField.x(), p.positionOnField.y(),
           p.positionOnField.x(), p.positionOnField.y() +
           (p.position == GoalPost::IS_LEFT ? -700 : 700),
           60, Drawings::solidPen, goalPerceptColor);
      MID_DOT("representation:GoalPercept:Image", p.positionInImage.x(), p.positionInImage.y(),
              goalPerceptColor, color);
      LINE("representation:GoalPercept:Image", p.positionInImage.x(), p.positionInImage.y(), p.positionInImage.x(), 0,
           5, Drawings::solidPen, goalPerceptColor);
      if(p.position == GoalPost::IS_LEFT)
        DRAWTEXT("representation:GoalPercept:Image", p.positionInImage.x() + 10, 40, 36, goalPerceptColor, "L");
      else if(p.position == GoalPost::IS_RIGHT)
        DRAWTEXT("representation:GoalPercept:Image", p.positionInImage.x() - 30, 40, 36, goalPerceptColor, "R");
    }
    else
    {
      CIRCLE("representation:GoalPercept:Field", p.positionOnField.x(), p.positionOnField.y(),
             50, 0, Drawings::solidPen, ColorRGBA(255, 0, 0), Drawings::solidBrush, goalPerceptColor);
      MID_DOT("representation:GoalPercept:Image", p.positionInImage.x(), p.positionInImage.y(),
              ColorRGBA(255, 0, 0), goalPerceptColor);
      LINE("representation:GoalPercept:Image", p.positionInImage.x(), p.positionInImage.y(), p.positionInImage.x(), 0,
           5, Drawings::dottedPen, goalPerceptColor);
    }
    // Sorry, no access to field dimensions here, so the dimensions are hard coded
    CYLINDER3D("representation:GoalPercept", p.positionOnField.x(), p.positionOnField.y(), 400, 0, 0, 0, 50, 800, goalPerceptColor);
    LINE3D("representation:GoalPercept", 0, 0, 1.f, p.positionOnField.x(), p.positionOnField.y(), 1.f, 2.f, goalPerceptColor);
  }
}
