/**
* @file LocalizationTeamBall.cpp
*
* Implementation of class LocalizationTeamBall
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
    COMPLEX_DRAWING("representation:LocalizationTeamBall",
    {
      CIRCLE("representation:LocalizationTeamBall", position.x, position.y, 30, 20, Drawings::ps_solid, ColorRGBA::red, Drawings::bs_null, ColorRGBA());
      CIRCLE("representation:LocalizationTeamBall", position.x, position.y, 300, 20, Drawings::ps_solid, ColorRGBA::red, Drawings::bs_null, ColorRGBA());
      ColorRGBA qualityColor = goalieHasObserved ? ColorRGBA::green : ColorRGBA::gray;
      for(int i=0; i<numOfObservers; i++)
      {
        CIRCLE("representation:LocalizationTeamBall", position.x + ((-2 + i) * 200), position.y + 400, 40,
               0, // pen width
               Drawings::ps_solid,
               qualityColor,
               Drawings::bs_solid,
               qualityColor);
      }
    });
  }
  
  DECLARE_DEBUG_DRAWING3D("representation:LocalizationTeamBall", "field",
  if(isValid)
  {
    ColorRGBA color = goalieHasObserved ? ColorRGBA(64, 128, 0) : ColorRGBA(128, 64, 0);
    CYLINDER3D("representation:LocalizationTeamBall", position.x, position.y, 0, 0, 0, 0, 50, numOfObservers*100, color);
  }
  );
}

