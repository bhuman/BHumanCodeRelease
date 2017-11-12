#include "PlayersFieldPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <algorithm>

void PlayersFieldPercept::draw() const
{
  DEBUG_DRAWING("representation:PlayersFieldPercept:field", "drawingOnField")
  {
    for(const auto& player : players)
    {
      if(!player.detectedFeet)
        continue;

      const float radius = std::max((player.leftOnField - player.rightOnField).norm() / 2.f, 50.f);
      DRAWTEXT("representation:PlayersFieldPercept:field", player.centerOnField.x() + radius, player.centerOnField.y(), 80, ColorRGBA::black, "Orientation Colored: " + std::to_string(player.orientation) + " (" + std::to_string(player.detectedOrientation) + ")");
      CIRCLE("representation:PlayersFieldPercept:field", player.centerOnField.x(), player.centerOnField.y(), radius, 0, Drawings::solidPen,
             !player.detectedJersey ? ColorRGBA::black : player.ownTeam ? ColorRGBA::blue : ColorRGBA::red, Drawings::noBrush, ColorRGBA());

      if(player.detectedOrientation == 2)
      {
        float x = std::cos(player.orientation) * radius;
        float y = std::sin(player.orientation) * radius;

        LINE("representation:PlayersFieldPercept:field", player.centerOnField.x() + x, player.centerOnField.y() + y, player.centerOnField.x(), player.centerOnField.y(), 5, Drawings::solidPen, ColorRGBA::yellow);
      }
      else if(player.detectedOrientation == 1)
      {
        float x = std::cos(player.orientation) * radius;
        float y = std::sin(player.orientation) * radius;

        LINE("representation:PlayersFieldPercept:field", player.centerOnField.x() + x, player.centerOnField.y() + y, player.centerOnField.x() - x, player.centerOnField.y() - y, 5, Drawings::solidPen, ColorRGBA::yellow);
      }
    }
  }

  DEBUG_DRAWING("representation:PlayersFieldPercept:image", "drawingOnImage")
  {
    for(const auto& player : players)
    {
      RECTANGLE("representation:PlayersFieldPercept:image", player.borderInImage[0], player.borderInImage[1], player.borderInImage[2], player.borderInImage[3], 4, Drawings::solidPen,
                !player.detectedJersey ? ColorRGBA::black : player.ownTeam ? ColorRGBA::blue : ColorRGBA::red);
    }
  }
}
