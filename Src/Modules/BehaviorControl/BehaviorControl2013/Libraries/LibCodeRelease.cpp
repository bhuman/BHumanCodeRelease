/**
* @file LibCodeRelease.cpp
*/

#include "../LibraryBase.h"

namespace Behavior2013
{
  #include "LibCodeRelease.h"
  
  LibCodeRelease::LibCodeRelease():
    angleToGoal(0.f)
  {}
  
  void LibCodeRelease::preProcess()
  {
    angleToGoal = (theRobotPose.invert() * Vector2<> (theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }

  void LibCodeRelease::postProcess()
  {
  }
  
  int LibCodeRelease::timeSinceBallWasSeen()
  {
    return theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  }
  
  bool LibCodeRelease::between(float value, float min, float max)
  {
    return value >= min && value <= max;
  }  
}