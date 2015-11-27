#include "PlayersPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <algorithm>

void PlayersPercept::draw() const
{
  DEBUG_DRAWING("representation:PlayersPercept:Image", "drawingOnImage")
  {
    for(const auto& player : players)
    {
      RECTANGLE("representation:PlayersPercept:Image", player.x1FeetOnly, player.y1, player.x2FeetOnly, player.y2, 4, Drawings::solidPen, ColorRGBA::white);

      RECTANGLE("representation:PlayersPercept:Image", player.x1, player.y1, player.x2, player.y2, 4, Drawings::solidPen,
                !player.detectedJersey ? ColorRGBA::black : player.ownTeam ? ColorRGBA::blue : ColorRGBA::red);

      if(player.fallen) // big blue cross indicating the obstacle is lying on the ground
      {
        CROSS("representation:PlayersPercept:Image", player.realCenterX, player.y2, 4, 4, Drawings::solidPen, ColorRGBA::blue);
      }

      if(player.detectedFeet) // red cross indicating the position of an obstacle on the field, if visible
      {
        CROSS("representation:PlayersPercept:Image", player.realCenterX, player.y2, 2, 4, Drawings::solidPen, ColorRGBA::red);
      }
    }
  }

  DEBUG_DRAWING("representation:PlayersPercept:Field", "drawingOnField")
  {
    for(const auto& player : players)
    {
      if(!player.detectedFeet)
        continue;
      const float radius = std::max((player.leftOnField - player.rightOnField).norm() / 2.f, 50.f);
      CIRCLE("representation:PlayersPercept:Field", player.centerOnField.x(), player.centerOnField.y(), radius, 0, Drawings::solidPen,
             !player.detectedJersey ? ColorRGBA::black : player.ownTeam ? ColorRGBA::blue : ColorRGBA::red, Drawings::noBrush, ColorRGBA());
    }
  }
}
