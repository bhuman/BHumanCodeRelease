/**
* @file RobotPercept.h
* @author Michel Bartsch
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(RobotPercept,
{
public:
  STREAMABLE(RobotBox,
  {,
    (int) x1,
    (int) y1,
    (int) x2,
    (int) y2,
    (int) realCenterX,
    (bool) lowerCamera,
    (bool) detectedJersey,
    (bool) detectedFeet,
    (bool) teamRed,
    (bool) fallen,
    (Vector2<>) pos,
  });

  /** Draws the position of the robot on the ground. */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:RobotPercept:Image", "drawingOnImage");

    for(std::vector<RobotBox>::const_iterator i = robots.begin(), end = robots.end(); i != end; ++i)
    {
      if(!i->detectedJersey)
      {
        RECTANGLE("representation:RobotPercept:Image", i->x1, i->y1, i->x2, i->y2, 4, Drawings::ps_solid, ColorClasses::black);
      }
      else
      {
        if(i->teamRed)
        {
          RECTANGLE("representation:RobotPercept:Image", i->x1, i->y1, i->x2, i->y2, 4, Drawings::ps_solid, ColorClasses::red);
        }
        else
        {
          RECTANGLE("representation:RobotPercept:Image", i->x1, i->y1, i->x2, i->y2, 4, Drawings::ps_solid, ColorClasses::blue);
        }
      }
      if(i->fallen)
      {
        CROSS("representation:RobotPercept:Image", i->realCenterX, i->y2, 4, 4, Drawings::ps_solid, ColorClasses::blue);
      }
      if(i->detectedFeet)
      {
        CROSS("representation:RobotPercept:Image", i->realCenterX, i->y2, 2, 4, Drawings::ps_solid, ColorClasses::red);
      }
    }
  },

  (std::vector<RobotBox>) robots,
});
