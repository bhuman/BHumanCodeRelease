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
      SystemCall::say("High");
  }
  else
  {
    if(theFrameInfo.getTimeSince(theFsrData.legInfo[Legs::left].hasPressure) > maxTimeWithoutPressure ||
       theFrameInfo.getTimeSince(theFsrData.legInfo[Legs::right].hasPressure) > maxTimeWithoutPressure ||
       (theFsrData.legInfo[Legs::left].totals <= minPressurePerFootToRegainContact &&
        theFsrData.legInfo[Legs::right].totals <= minPressurePerFootToRegainContact))
      lastTimeWithoutPressure = theFrameInfo.time;
    groundContactState.contact = theFrameInfo.getTimeSince(lastTimeWithoutPressure) > minTimeWithPressure;
    if(groundContactState.contact && SystemCall::getMode() == SystemCall::physicalRobot)
      SystemCall::say("Ground");
  }
}
