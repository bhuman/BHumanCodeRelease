/**
 * @file FrictionLearner.cpp
 *
 * This file does something very hacky
 *
 * @author Beeernd
 */

#include "DynamicGlobalOptionsProvider.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(DynamicGlobalOptionsProvider, cognitionInfrastructure)

void DynamicGlobalOptionsProvider::update(GlobalOptions& options)
{
  float buffer = options.slowWalk ? bufferSize : 0;

  options.slowWalk = false;
  for(Vector4f area : areas)
  {
    if(mirror)
      area *= -1;

    if(!theGameInfo.firstHalf)
      area *= -1;

    DEBUG_DRAWING("module:DynamicGlobalOptionsProvider:rects", "drawingOnField")
    {
      RECTANGLE("module:DynamicGlobalOptionsProvider:rects", area[0], area[1], area[2], area[3],  10, Drawings::solidPen, ColorRGBA::magenta);
    }

    if(Rangef(std::min(area[0], area[2]) - buffer, std::max(area[0], area[2]) + buffer).isInside(theRobotPose.translation.x()) && Rangef(std::min(area[1], area[3]) - buffer, std::max(area[1], area[3]) + buffer).isInside(theRobotPose.translation.y()))
    {
      options.slowWalk = true;
      return;
    }
  }
}
