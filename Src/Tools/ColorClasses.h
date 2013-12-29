/**
* @file ColorClasses.h
* Declaration of class ColorClasses
*
* @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
*/

#pragma once

#include "Tools/Enum.h"

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
    orange,                 /*<! ball */
    yellow,                 /*<! yellow goal */
    blue,                   /*<! color of blue robots> */
    white,                  /*<! lines */
    green,                  /*<! field */
    black,                  /*<! most probably: nothing */
    red                     /*<! color of red robots> */
  );

  /**
  * The method returns prototypical color values for a color class for visualization.
  * @param colorClass The color class.
  * @param y The Y channel of the prototypical color.
  * @param cr The Cr channel of the prototypical color.
  * @param cb The Cb channel of the prototypical color.
  */
  static inline void getColorClassColor(ColorClasses::Color colorClass, unsigned char& y, unsigned char& cr, unsigned char& cb)
  {
    switch(colorClass)
    {
    case ColorClasses::white:
      y = 255;
      cr = 127;
      cb = 127;
      break;
    case ColorClasses::green:
      y = 180;
      cr = 0;
      cb = 0;
      break;
    case ColorClasses::orange:
      y = 164;
      cr = 255;
      cb = 0;
      break;
    case ColorClasses::yellow:
      y = 255;
      cr = 170;
      cb = 0;
      break;
    case ColorClasses::blue:
      y = 60;
      cr = 80;
      cb = 255;
      break;
    case ColorClasses::black:
      y = 0;
      cr = 127;
      cb = 127;
      break;
    case ColorClasses::red:
      y = 105;
      cr = 234;
      cb = 127;
      break;
    default:
      y = 70;
      cr = 127;
      cb = 127;
    }
  }
};
