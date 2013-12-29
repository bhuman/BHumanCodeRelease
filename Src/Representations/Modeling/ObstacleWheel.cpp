/*
 * File:   ObstacleWheel.cpp
 * Author: arne
 *
 * Created on March 4, 2013, 4:23 PM
 */

#include "ObstacleWheel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Vector2.h"

void ObstacleWheel::draw() const
{

  DECLARE_DEBUG_DRAWING("representation:ObstacleWheel:wheel", "drawingOnField");
  for(const Cone& cone : cones)
  {
    drawCone(cone);
  }

  CIRCLE("representation:ObstacleWheel:wheel", 0, 0, wheelRadius, 1,
         Drawings::ps_solid, ColorClasses::black, Drawings::bs_null, ColorClasses::black);
}

void ObstacleWheel::drawCone(const Cone& cone) const
{
  float rightAngle = cone.angle;
  float leftAngle = cone.angle + coneWidth;

  Vector2<float> leftBorder(static_cast<float>(wheelRadius), 0.f);
  Vector2<float> rightBorder(leftBorder);
  leftBorder = leftBorder.rotate(leftAngle);
  rightBorder = rightBorder.rotate(rightAngle);

  LINE("representation:ObstacleWheel:wheel", 0, 0, leftBorder.x, leftBorder.y,
           1, Drawings::ps_solid, ColorRGBA(0x00, 0x00, 0xFF, 0x0F));


  if(cone.spot.seenCount > 0)
  {
    Vector2<float> left(cone.distance, 0);
    Vector2<float> right(cone.distance, 0);
    left.rotate(leftAngle);
    right.rotate(rightAngle);

    ColorClasses::Color col = ColorClasses::green;
    if(cone.hasObstacle)
    {
      col = ColorClasses::red;
    }
    LINE("representation:ObstacleWheel:wheel", left.x, left.y, right.x, right.y,
         8, Drawings::ps_solid, col);
  }

}