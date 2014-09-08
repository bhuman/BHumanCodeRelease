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
    (int) x1, // left border in the image
    (int) y1, // top border in the image
    (int) x2, // right border in the image
    (int) y2, // bottom border in the image
    (int) realCenterX, // the real horizontal center of the robot, not allways middle between x1 and x2
    (int) x1FeetOnly, // left border of only the robots feet in the image
    (int) x2FeetOnly, // right border of only the robots feet in the image
    (bool) lowerCamera, // true, if the robot was spotted in the lower camera
    (bool) detectedJersey, // true, if a jersey was found
    (bool) detectedFeet, // true, if the bottom of the obstacle was visible; never calculate position on field without detectedFeet!
    (bool) teamRed, // true, if a detected jersey was red
    (bool) fallen, // true, if the obstacle seems to lay on the field
  });

  /** Draws boxes around obstacles. */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:RobotPercept:Image", "drawingOnImage");

    for(std::vector<RobotBox>::const_iterator i = robots.begin(), end = robots.end(); i != end; ++i)
    {
      RECTANGLE("representation:RobotPercept:Image", i->x1FeetOnly, i->y1, i->x2FeetOnly, i->y2, 4, Drawings::ps_solid, ColorRGBA::white);
      if(!i->detectedJersey) // black box as long as no jersey was found, red or blue boxes representing jersey else
      {
        RECTANGLE("representation:RobotPercept:Image", i->x1, i->y1, i->x2, i->y2, 4, Drawings::ps_solid, ColorRGBA::black);
      }
      else
      {
        if(i->teamRed)
        {
          RECTANGLE("representation:RobotPercept:Image", i->x1, i->y1, i->x2, i->y2, 4, Drawings::ps_solid, ColorRGBA::red);
        }
        else
        {
          RECTANGLE("representation:RobotPercept:Image", i->x1, i->y1, i->x2, i->y2, 4, Drawings::ps_solid, ColorRGBA::blue);
        }
      }
      if(i->fallen) // big blue cross indicating the obstacle is lying on the ground
      {
        CROSS("representation:RobotPercept:Image", i->realCenterX, i->y2, 4, 4, Drawings::ps_solid, ColorRGBA::blue);
      }
      if(i->detectedFeet) // red cross indicating the position of an obstacle on the field, if visible
      {
        CROSS("representation:RobotPercept:Image", i->realCenterX, i->y2, 2, 4, Drawings::ps_solid, ColorRGBA::red);
      }
    }
  },

  (std::vector<RobotBox>) robots,
});
