/**
 * @file ColorClasses.h
 * Declaration of namespace ColorClasses
 *
 * @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
 */

#pragma once

#include "ColorRGBA.h"
#include "Tools/Enum.h"

/**
 * @namespace ColorClasses
 *
 * Namespace for color class functions.
 *
 * @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
 */
namespace ColorClasses
{
  ENUM(Color,
  {,
    none,   /*<! all other objects */
    white,  /*<! lines */
    green,  /*<! field */
    blue,   /*<! color of blue robots */
    red,    /*<! color of red robots */
    orange, /*<! ball */
    black,  /*<! most probably: nothing */
  });

  /**
   * The method returns prototypical color values for a color class for visualization.
   * @param colorClass The color class.
   * @return A reference to the corresponding RGB color.
   */
  inline const ColorRGBA& getColorRGBA(Color colorClass)
  {
    static const ColorRGBA colors[Color::numOfColors] =
    {
      ColorRGBA::gray,
      ColorRGBA::white,
      ColorRGBA::green,
      ColorRGBA::blue,
      ColorRGBA::red,
      ColorRGBA::orange,
      ColorRGBA::black
    };
    return colors[colorClass];
  }
};
