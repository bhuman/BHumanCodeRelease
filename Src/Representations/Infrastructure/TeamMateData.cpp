/**
 * @file TeamMateData.cpp
 * Declaration of a class representing information about the teammates.
 * @author Colin Graf, Alexis Tsogias
 */

#include "TeamMateData.h"

void TeamMateData::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:TeamMateData", "drawingOnField");
  for(int i = 1; i < numOfPlayers; ++i)
  {
    if(timeStamps[i])
    {
      ColorRGBA posCol;
      if(isFullyActive[i])
      {
        posCol = ColorRGBA(0, 255, 0);
      }
      else if(isActive[i])
      {
        posCol = ColorRGBA(255, 255, 0);
      }
      else
      {
        posCol = ColorRGBA(255, 0, 0);
      }

      const Vector2<>& rPos = robotPoses[i].translation;
      float radius = robotPoses[i].deviation;
      if(radius < 50.f)
      {
        radius = 50.f;
      }

      Vector2<> dirPos = robotPoses[i] * Vector2<>(radius, 0);
       // Circle arround Player
      CIRCLE("representation:TeamMateData", rPos.x, rPos.y, radius, 20, Drawings::ps_solid,
             posCol, Drawings::bs_null, ColorClasses::white);
      // Direction of the Robot
      LINE("representation:TeamMateData", rPos.x, rPos.y, dirPos.x, dirPos.y, 20,
           Drawings::ps_solid, posCol);
      // Player number
      DRAWTEXT("representation:TeamMateData", rPos.x, rPos.y, 150, ColorClasses::black, i);
      // Role
      DRAWTEXT("representation:TeamMateData", rPos.x + 200, rPos.y - 200, 150,
               ColorClasses::black, BehaviorStatus::getName(behaviorStatus[i].role));
      Vector2<> bPos = robotPoses[i] * ballModels[i].estimate.position;
      ColorRGBA ballCol;
      if(currentTimestamp - ballModels[i].timeWhenLastSeen < networkTimeout / 2)
      {
        ballCol = ColorRGBA(0, 255, 0);
      }
      else if(currentTimestamp - ballModels[i].timeWhenLastSeen < networkTimeout)
      {
        ballCol = ColorRGBA(255, 255, 0);
      }
      else
      {
        ballCol = ColorRGBA(255, 0, 0);
      }

      // Ball position
      CIRCLE("representation:TeamMateData", bPos.x, bPos.y, 50, 20, Drawings::ps_solid,
             ballCol, Drawings::bs_solid, ballCol);
      //Line from Robot to Ball
      LINE("representation:TeamMateData", rPos.x, rPos.y, bPos.x, bPos.y, 20, Drawings::ps_dash, ballCol);
    }
  }
}
