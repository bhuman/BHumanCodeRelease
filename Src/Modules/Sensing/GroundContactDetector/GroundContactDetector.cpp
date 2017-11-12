/**
 * @file GroundContactDetector.h
 * Implementation of a module that detects ground contact based on FSR measurements.
 * This changes the semantics of ground contact, because this module does not
 * care whether the robot is shaking heavily while there is still foot contact.
 * @author Thomas Röfer
 */

#include "GroundContactDetector.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(GroundContactDetector, sensing)

void GroundContactDetector::update(GroundContactState& groundContactState)
{
  if(groundContactState.contact)
  {
    if(theFsrSensorData.totals[Legs::left] + theFsrSensorData.totals[Legs::right] >= minPressureToKeepContact)
      lastTimeWithPressure = theFrameInfo.time;
    groundContactState.contact = theFrameInfo.getTimeSince(lastTimeWithPressure) < maxTimeWithoutPressure;
    if(!groundContactState.contact && SystemCall::getMode() == SystemCall::physicalRobot)
      SystemCall::playSound("high.wav");
  }
  else
  {
    if(std::abs(theInertialSensorData.gyro.y()) > maxGyroYToRegainContact
       || theFsrSensorData.totals[Legs::left] + theFsrSensorData.totals[Legs::right] < minPressureToRegainContact
       || theFsrSensorData.totals[Legs::left] < minPressurePerFootToRegainContact
       || theFsrSensorData.totals[Legs::right] < minPressurePerFootToRegainContact)
      lastTimeWithoutPressure = theFrameInfo.time;
    groundContactState.contact = theFrameInfo.getTimeSince(lastTimeWithoutPressure) > minTimeWithPressure;
    if(groundContactState.contact && SystemCall::getMode() == SystemCall::physicalRobot)
      SystemCall::playSound("ground.wav");
  }
}
