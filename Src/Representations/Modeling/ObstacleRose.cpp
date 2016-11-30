//
//  ObstacleRose.cpp
//  B-Human
//
//  Created by Andre Lübken on 14.12.15.
//
//

#include "ObstacleRose.h"
#include "Tools/Debugging/DebugDrawings.h"

ObstacleRose::ObstacleRose()
{
  updateSectorSettings(frameCount, sectorCount);
}

// rebuild the sector model, if settings change
void ObstacleRose::updateSectorSettings(const unsigned frameCount, const unsigned sectorCount)
{
  obstacleSectors.clear();

  for(unsigned i = 1; i <= sectorCount; ++i)
  {
    obstacleSectors.push_back(ObstacleSector(pi2 / sectorCount * (i - 1), pi2 / sectorCount * i, frameCount));
  }
}

void ObstacleRose::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ObstacleRose:sectors", "drawingOnField");
  {
    for(auto& sector : obstacleSectors)
    {
      unsigned char intensity = 0;
      intensity = static_cast<unsigned char>(std::count(sector.blocked, std::end(sector.blocked), true) * (255 / floor(FRAME_COUNT)));

      float polarX = DRAW_SECTOR_LINE_LENGTH * cosf(sector.startBearing);
      float polarY = DRAW_SECTOR_LINE_LENGTH * sinf(sector.startBearing);

      LINE("representation:ObstacleRose:sectors",
           pose.translation.x(), pose.translation.y(), pose.translation.x() + polarX, pose.translation.y() + polarY,
           8, Drawings::solidPen, ColorRGBA::cyan);

      if(intensity > 0)
      {
        // TODO: cant figure out filling of elements. do a poor mans filled arc
        for(int drawingDistance = DRAW_SECTOR_LINE_LENGTH; drawingDistance > 100; drawingDistance -= 50)
        {
          ARC("representation:ObstacleRose:sectors",
              pose.translation.x(), pose.translation.y(), drawingDistance, sector.startBearing, sector.stopBearing - sector.startBearing,
              20, Drawings::PenStyle::solidPen, ColorRGBA(255, 80, 0, intensity), Drawings::solidBrush, ColorRGBA::red);
        }
      }
    }
  }
}
