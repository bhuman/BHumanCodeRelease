/**
 * @file LocalizationTeamBall.cpp
 *
 * Implementation of struct LocalizationTeamBall
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#include "LocalizationTeamBall.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void LocalizationTeamBall::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:LocalizationTeamBall", "drawingOnField");
  if(isValid)
  {
    COMPLEX_DRAWING("representation:LocalizationTeamBall")
    {
      CIRCLE("representation:LocalizationTeamBall", position.x(), position.y(), 30, 20, Drawings::solidPen, ColorRGBA::red, Drawings::noBrush, ColorRGBA());
      CIRCLE("representation:LocalizationTeamBall", position.x(), position.y(), 300, 20, Drawings::solidPen, ColorRGBA::red, Drawings::noBrush, ColorRGBA());
      ColorRGBA qualityColor = goalieHasObserved ? ColorRGBA::green : ColorRGBA::gray;
      for(int i=0; i<numOfObservers; i++)
      {
        CIRCLE("representation:LocalizationTeamBall", position.x() + ((-2 + i) * 200), position.y() + 400, 40,
               0, // pen width
               Drawings::solidPen, qualityColor,
               Drawings::solidBrush, qualityColor);
      }
    }
  }

  DEBUG_DRAWING3D("representation:LocalizationTeamBall", "field")
  {
    if(isValid)
    {
      ColorRGBA color = goalieHasObserved ? ColorRGBA(64, 128, 0) : ColorRGBA(128, 64, 0);
      CYLINDER3D("representation:LocalizationTeamBall", position.x(), position.y(), 0, 0, 0, 0, 50, numOfObservers * 100, color);
    }
  }
}
