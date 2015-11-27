/**
* @file VerifiedCenterCircle
*
* Description follows
*
* @author Tim Laue
*/

#pragma once

#include "SelfLocatorBase.h"
#include "Tools/Debugging/DebugDrawings.h"


/**
* @class VerifiedCenterCircle
*
* The class
*/
class VerifiedCenterCircle
{
  class Perception
  {
  public:
    Perception():timestamp(0), seenCount(0)
    {}
    
    Vector2f currentPosition(const Pose2f& currentOdometry) const
    {
      const Pose2f odometryOffset =  odometryAtTimestamp - currentOdometry;
      return odometryOffset * position;
    }
    
    Vector2f position;
    int timestamp;
    int seenCount;
    Pose2f odometryAtTimestamp;
  };
 
  bool centerCircleIsValid = false;
  Perception centerCircle;
  
public:
  void reset()
  {
    centerCircleIsValid = false;
    centerCircle.timestamp = 0;
  }
  
  void updateCenterCircle(const LinePercept& theLinePercept, const FrameInfo& theFrameInfo,
                          const OdometryData& theOdometryData)
  {
    if(theLinePercept.circle.found)
    {
      // The perception has not been filled yet or is too old:
      if(centerCircle.timestamp == 0 ||
        theFrameInfo.getTimeSince(centerCircle.timestamp) > 10000)
      {
        centerCircle.position = theLinePercept.circle.pos;
        centerCircle.timestamp = theFrameInfo.time;
        centerCircle.seenCount = 1;
        centerCircle.odometryAtTimestamp = theOdometryData;
      }
      else
      {
        // Try to merge new perception with old one
        Vector2f currentPosition = centerCircle.currentPosition(theOdometryData);
        // TODO: Better comparison, this is just a test
        if((currentPosition - theLinePercept.circle.pos).norm() <= 1000)
        {
          // TODO: Better merging formula, this is just a test
          centerCircle.position = theLinePercept.circle.pos;
          centerCircle.timestamp = theFrameInfo.time;
          centerCircle.seenCount++;
          centerCircle.odometryAtTimestamp = theOdometryData;
        }
      }
    }
    // If merging was not possible, the perception will become outdated after
    // a few seconds and become replaced by a new one
    centerCircleIsValid = theFrameInfo.getTimeSince(centerCircle.timestamp) < 10000;
  }
  
  bool isGoalpostCompatibleToCenterCircle(const Vector2f& goalpostPos, const OdometryData& theOdometryData) const
  {
    if(centerCircleIsValid && centerCircle.timestamp != 0 && centerCircle.seenCount > 4)
      return (goalpostPos - centerCircle.currentPosition(theOdometryData)).norm() > 3000.f;
    else
      return true;
  }
  
  void draw(const OdometryData& theOdometryData)
  {
    if(centerCircleIsValid)
    {
      const ColorRGBA drawColor(20,255,255);
      const Vector2f circlePos = centerCircle.currentPosition(theOdometryData);
      CIRCLE("module:SelfLocator:verifiedCenterCircle", circlePos.x(), circlePos.y(),
             500, 20, Drawings::solidPen, drawColor, Drawings::noBrush, drawColor);
      CROSS("module:SelfLocator:verifiedCenterCircle", circlePos.x(), circlePos.y(),
            100, 20, Drawings::solidPen, drawColor);
    }
  }
};
