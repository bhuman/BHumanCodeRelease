/**
* @file ColorClasses.h
* Declaration of class ColorClasses
*
* @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
*/

#pragma once

#include "Tools/Enum.h"
#include "Tools/Debugging/DebugDrawings.h"

/**
* @class ColorClasses
*
* Static class for color class functions.
*
* @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
*/
class ColorClasses
{
public:
  ENUM(Color,
    none,                   /*<! all other objects */
    white,                  /*<! lines */
    green,                  /*<! field */
    blue,                   /*<! color of blue robots */
    red,                    /*<! color of red robots */
    orange,                 /*<! ball */
    yellow,                 /*<! yellow goal */
    black                   /*<! most probably: nothing */
  );

  /**
  * The method returns prototypical color values for a color class for visualization.
  * @param colorClass The color class.
  * @return A reference to the corresponding RGB color.
  */
  static const ColorRGBA& getColorRGBA(ColorClasses::Color colorClass)
  {
    static const ColorRGBA colors[ColorClasses::numOfColors] =
    {
      ColorRGBA::gray,
      ColorRGBA::white,
      ColorRGBA::green,
      ColorRGBA::blue,
      ColorRGBA::red,
      ColorRGBA::orange,
      ColorRGBA::yellow,
      ColorRGBA::black
    };
    return colors[colorClass];
  }
};
