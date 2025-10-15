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
  // Ground contact can only be decide after sensor data was received.
  if(!theFrameInfo.time)
    return;

  if(groundContactState.contact)
  {
    if(theSolePressureState.legInfo[Legs::left].hasPressure || theSolePressureState.legInfo[Legs::right].hasPressure)
      lastTimeWithPressure = theFrameInfo.time;
    groundContactState.contact = theFrameInfo.getTimeSince(lastTimeWithPressure) < maxTimeWithoutPressure;
    if(!groundContactState.contact && SystemCall::getMode() == SystemCall::physicalRobot)
      lastTimeHigh = theFrameInfo.time;
    groundContactState.lastGroundContactTimestamp = theFrameInfo.time;
  }
  else
  {
    if((!theSolePressureState.legInfo[Legs::left].hasPressure && theFrameInfo.getTimeSince(theSolePressureState.legInfo[Legs::left].hasPressureSince) > maxTimeWithoutPressure) ||
       (!theSolePressureState.legInfo[Legs::right].hasPressure && theFrameInfo.getTimeSince(theSolePressureState.legInfo[Legs::right].hasPressureSince) > maxTimeWithoutPressure))
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
