#include "PlayersImagePercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <algorithm>

void PlayersImagePercept::draw() const
{
  DEBUG_DRAWING("representation:PlayersImagePercept:image", "drawingOnImage")
  {
    for(const auto& player : players)
    {
      RECTANGLE("representation:PlayersImagePercept:image", player.x1FeetOnly, player.y1, player.x2FeetOnly, player.y2, 4, Drawings::solidPen, ColorRGBA::white);

      if(player.fallen) // big blue cross indicating the obstacle is lying on the ground
      {
        CROSS("representation:PlayersImagePercept:image", player.realCenterX, player.y2, 4, 4, Drawings::solidPen, ColorRGBA::blue);
      }

      if(player.detectedFeet) // red cross indicating the position of an obstacle on the field, if visible
      {
        CROSS("representation:PlayersImagePercept:image", player.realCenterX, player.y2, 2, 4, Drawings::solidPen, ColorRGBA::red);
      }
    }
  }

  DEBUG_DRAWING("representation:PlayersImagePercept:spots", "drawingOnImage")
  {
    for(const PlayerSpot& spot : spots)
    {
      RECTANGLE("representation:PlayersImagePercept:spots", spot.border0, spot.border1, spot.border2, spot.border3, 4, Drawings::solidPen, ColorRGBA::yellow);
      DRAWTEXT("representation:PlayersImagePercept:spots", spot.border0, spot.border3 + 10, 8, ColorRGBA::black, "Size: " + std::to_string(spot.size) + " | MinFilled: " + std::to_string(spot.minFilled));
    }
  }
}
