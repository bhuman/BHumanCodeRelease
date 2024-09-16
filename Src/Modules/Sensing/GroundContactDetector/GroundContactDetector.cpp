/**
 * @file GroundContactDetector.cpp
 * Implementation of a module that detects ground contact based on FSR measurements.
 * @author Thomas Röfer
 */

#include "GroundContactDetector.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(GroundContactDetector);

void GroundContactDetector::update(GroundContactState& groundContactState)
{
  if(groundContactState.contact)
  {
    if(theFsrData.legInfo[Legs::left].hasPressure == theFsrData.lastUpdateTimestamp || theFsrData.legInfo[Legs::right].hasPressure == theFsrData.lastUpdateTimestamp)
      lastTimeWithPressure = theFrameInfo.time;
    groundContactState.contact = theFrameInfo.getTimeSince(lastTimeWithPressure) < maxTimeWithoutPressure;
    if(!groundContactState.contact && SystemCall::getMode() == SystemCall::physicalRobot)
      lastTimeHigh = theFrameInfo.time;
    groundContactState.lastGroundContactTimestamp = theFrameInfo.time;
  }
  else
  {
    if(theFrameInfo.getTimeSince(theFsrData.legInfo[Legs::left].hasPressure) > maxTimeWithoutPressure ||
       theFrameInfo.getTimeSince(theFsrData.legInfo[Legs::right].hasPressure) > maxTimeWithoutPressure)
      lastTimeWithoutPressure = theFrameInfo.time;
    groundContactState.contact = theFrameInfo.getTimeSince(lastTimeWithoutPressure) > minTimeWithPressure;
    if(groundContactState.contact && SystemCall::getMode() == SystemCall::physicalRobot)
      lastTimeGround = theFrameInfo.time;
  }

  if(theFrameInfo.getTimeSince(lastTimeSay) > minTimeSay && (lastTimeHigh > lastTimeSay || lastTimeGround > lastTimeSay))
  {
    if(lastTimeHigh > lastTimeGround)
      SystemCall::say("High");
    else
      SystemCall::say("Ground");
    lastTimeSay = theFrameInfo.time;
  }
}
