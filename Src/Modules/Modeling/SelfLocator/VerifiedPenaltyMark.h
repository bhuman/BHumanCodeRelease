/**
* @file VerifiedPenaltyMark
*
* Description follows
*
* @author Tim Laue
*/

#pragma once

#include "SelfLocatorBase.h"
#include "Tools/Debugging/DebugDrawings.h"


/**
* @class VerifiedPenaltyMark
*
* The class
*/
class VerifiedPenaltyMark
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
 
  bool penaltyMarkIsValid = false;
  Perception penaltyMark;
  
public:
  void reset()
  {
    penaltyMarkIsValid = false;
    penaltyMark.timestamp = 0;
  }
  
  void updatePenaltyMark(const PenaltyMarkPercept& thePenaltyMarkPercept, const FrameInfo& theFrameInfo,
                          const OdometryData& theOdometryData)
  {
    if(thePenaltyMarkPercept.timeLastSeen == theFrameInfo.time)
    {
      // The perception has not been filled yet or is too old:
      if(penaltyMark.timestamp == 0 ||
        theFrameInfo.getTimeSince(penaltyMark.timestamp) > 10000)
      {
        penaltyMark.position = thePenaltyMarkPercept.positionOnField;
        penaltyMark.timestamp = theFrameInfo.time;
        penaltyMark.seenCount = 1;
        penaltyMark.odometryAtTimestamp = theOdometryData;
      }
      else
      {
        // Try to merge new perception with old one
        Vector2f currentPosition = penaltyMark.currentPosition(theOdometryData);
        // TODO: Better comparison, this is just a test
        if((currentPosition - thePenaltyMarkPercept.positionOnField).norm() <= 1000)
        {
          // TODO: Better merging formula, this is just a test
          penaltyMark.position = thePenaltyMarkPercept.positionOnField;
          penaltyMark.timestamp = theFrameInfo.time;
          penaltyMark.seenCount++;
          penaltyMark.odometryAtTimestamp = theOdometryData;
        }
      }
    }
    // If merging was not possible, the perception will become outdated after
    // a few seconds and become replaced by a new one
    penaltyMarkIsValid = theFrameInfo.getTimeSince(penaltyMark.timestamp) < 10000;
  }
  
  bool isGoalpostCompatibleToPenaltyMark(const Vector2f& goalpostPos, const OdometryData& theOdometryData) const
  {
    const float exclusionIntervalStart = 2500.f;
    const float exclusionIntervalEnd = 5200.f;
    const float distanceFromMarkToGoal = (goalpostPos - penaltyMark.currentPosition(theOdometryData)).norm();
    if(penaltyMarkIsValid && penaltyMark.timestamp != 0 && penaltyMark.seenCount > 4)
      // close and very far goals are OK, something at the side is not...
      return (distanceFromMarkToGoal < exclusionIntervalStart) || (distanceFromMarkToGoal > exclusionIntervalEnd);
    else
      return true;
  }
  
  void draw(const OdometryData& theOdometryData)
  {
    if(penaltyMarkIsValid)
    {
      const ColorRGBA drawColor(20,255,255);
      const Vector2f markPos = penaltyMark.currentPosition(theOdometryData);
      CROSS("module:SelfLocator:verifiedPenaltyMark", markPos.x(), markPos.y(),
            400, 20, Drawings::solidPen, drawColor);
    }
  }
};
