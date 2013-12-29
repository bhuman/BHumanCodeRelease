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
  DECLARE_DEBUG_DRAWING("representation:GoalPercept:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:GoalPercept:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:GoalPercept", "origin");
  TRANSLATE3D("representation:GoalPercept", 0, 0, -230);
  ColorRGBA color = ColorClasses::yellow;
  for(unsigned int i = 0; i < goalPosts.size(); ++i)
  {
    const GoalPost& p = goalPosts.at(i);
    if(p.position != GoalPost::IS_UNKNOWN)
    {
      CIRCLE("representation:GoalPercept:Field", p.positionOnField.x, p.positionOnField.y,
             50, 0, Drawings::ps_solid, ColorClasses::white, Drawings::bs_solid, color);
      LINE("representation:GoalPercept:Field", p.positionOnField.x, p.positionOnField.y,
           p.positionOnField.x, p.positionOnField.y +
           (p.position == GoalPost::IS_LEFT ? -700 : 700),
           60, Drawings::ps_solid, color);
      MID_DOT("representation:GoalPercept:Image", p.positionInImage.x, p.positionInImage.y,
              ColorClasses::white, color);
      LINE("representation:GoalPercept:Image", p.positionInImage.x, p.positionInImage.y, p.positionInImage.x, 0,
           5, Drawings::ps_solid, color);
      // Sorry, no access to field dimensions here, so the dimensions are hard coded
      CYLINDER3D("representation:GoalPercept", p.positionOnField.x, p.positionOnField.y, 400, 0, 0, 0, 50, 800, color);
    }
    else
    {
      CIRCLE("representation:GoalPercept:Field", p.positionOnField.x, p.positionOnField.y,
             50, 0, Drawings::ps_solid, ColorRGBA(255, 0, 0), Drawings::bs_solid, color);
      MID_DOT("representation:GoalPercept:Image", p.positionInImage.x, p.positionInImage.y,
              ColorRGBA(255, 0, 0), color);
      LINE("representation:GoalPercept:Image", p.positionInImage.x, p.positionInImage.y, p.positionInImage.x, 0,
           5, Drawings::ps_dot, color);
      // Sorry, no access to field dimensions here, so the dimensions are hard coded
      CYLINDER3D("representation:GoalPercept", p.positionOnField.x, p.positionOnField.y, 400, 0, 0, 0, 50, 800, color);
    }
  }
}
