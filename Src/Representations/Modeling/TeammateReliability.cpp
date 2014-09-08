/**
* @file TeammateReliability.cpp
*/

#include "TeammateReliability.h"

void TeammateReliability::draw()
{
  DECLARE_DEBUG_DRAWING("representation:TeammateReliability", "drawingOnField");
  int x = -5000;
  for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; ++i, x += 1700)
  {
    ColorRGBA color;
    switch(states[i])
    {
      case UNKNOWN:
      case CHECKING:
        color = ColorRGBA::red;
        break;
      case UNRELIABLE:
        color = ColorRGBA::orange;
        break;
      case OK:
        color = ColorRGBA::yellow;
        break;
      case GOOD:
          color = ColorRGBA::green;
        break;
    }
    DRAWTEXT("representation:TeammateReliability", x, 4000, 230, ColorRGBA::black, "Player " << i << ":");
    DRAWTEXT("representation:TeammateReliability", x, 3700, 230, color, getName(states[i]));
  }
}