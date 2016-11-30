#include "PlayersPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <algorithm>

void PlayersPercept::draw() const
{
  DEBUG_DRAWING("representation:PlayersPercept:image", "drawingOnImage")
  {
    for(const auto& player : players)
    {
      RECTANGLE("representation:PlayersPercept:image", player.x1FeetOnly, player.y1, player.x2FeetOnly, player.y2, 4, Drawings::solidPen, ColorRGBA::white);

      RECTANGLE("representation:PlayersPercept:image", player.x1, player.y1, player.x2, player.y2, 4, Drawings::solidPen,
                !player.detectedJersey ? ColorRGBA::black : player.ownTeam ? ColorRGBA::blue : ColorRGBA::red);

      if(player.fallen) // big blue cross indicating the obstacle is lying on the ground
      {
        CROSS("representation:PlayersPercept:image", player.realCenterX, player.y2, 4, 4, Drawings::solidPen, ColorRGBA::blue);
      }

      if(player.detectedFeet) // red cross indicating the position of an obstacle on the field, if visible
      {
        CROSS("representation:PlayersPercept:image", player.realCenterX, player.y2, 2, 4, Drawings::solidPen, ColorRGBA::red);
      }
    }
  }

  DEBUG_DRAWING("representation:PlayersPercept:field", "drawingOnField")
  {
    for(const auto& player : players)
    {
      if(!player.detectedFeet)
        continue;
      const float radius = std::max((player.leftOnField - player.rightOnField).norm() / 2.f, 50.f);
      CIRCLE("representation:PlayersPercept:field", player.centerOnField.x(), player.centerOnField.y(), radius, 0, Drawings::solidPen,
             !player.detectedJersey ? ColorRGBA::black : player.ownTeam ? ColorRGBA::blue : ColorRGBA::red, Drawings::noBrush, ColorRGBA());
    }
  }

  DEBUG_DRAWING("representation:PlayersPercept:spots", "drawingOnImage")
  {
    for(const PlayerSpot& spot : spots)
    {
      RECTANGLE("representation:PlayersPercept:spots", spot.border0, spot.border1, spot.border2, spot.border3, 4, Drawings::solidPen, ColorRGBA::yellow);
      DRAWTEXT("representation:PlayersPercept:spots", spot.border0, spot.border3 + 10, 8, ColorRGBA::black, "Size: " + std::to_string(spot.size) + " | MinFilled: " + std::to_string(spot.minFilled));
    }

    for(const Player& p : players)
    {
      RECTANGLE("representation:PlayersPercept:spots", p._debugJerseyScanX - 5, p._debugJerseyScanY - 5, p._debugJerseyScanX + 5, p._debugJerseyScanY + 5, 2, Drawings::solidPen, ColorRGBA::blue);
    }
  }
}
