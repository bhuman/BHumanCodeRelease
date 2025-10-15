/**
 * @file InternalStateProvider.cpp
 *
 * This file implements a module that writes the internal state
 * to a file, which is read by the DeployDialog.
 *
 * @author Thomas Röfer
 */

#include "InternalStateProvider.h"
#include "Streaming/OutStreams.h"

MAKE_MODULE(InternalStateProvider);

InternalStateProvider::InternalStateProvider()
{
#ifdef TARGET_ROBOT
  setPriority(writerPriority);
  start(this, &InternalStateProvider::writer);
#endif
}

InternalStateProvider::~InternalStateProvider()
{
#ifdef TARGET_ROBOT
  announceStop();
  writeNow.post();
  stop();
#endif
}

void InternalStateProvider::update(InternalState& theInternalState)
{
  if(theFrameInfo.getTimeSince(timeWhenLastWritten) >= writeDelay)
  {
    {
      SYNC;
      theInternalState.batteryLevel = theSystemSensorData.batteryLevel;
      theInternalState.batteryCharging = theSystemSensorData.batteryCharging;
      const Joints::Joint jointWithMaxTemperature = static_cast<Joints::Joint>(std::distance(theJointSensorData.temperatures.begin(), std::max_element(theJointSensorData.temperatures.begin(), theJointSensorData.temperatures.end())));
      theInternalState.maxTemperature = static_cast<float>(theJointSensorData.temperatures[jointWithMaxTemperature]);
    }
    writeNow.post();
    timeWhenLastWritten = theFrameInfo.time;
  }
}

void InternalStateProvider::writer()
{
  InternalState internalState;
  while(writeNow.wait() && isRunning())
  {
    {
      SYNC;
      internalState = theInternalState;
    }
    OutTextFile stream("/var/volatile/tmp/internalState.txt");
    if(stream.exists())
      stream << internalState.batteryLevel << static_cast<int>(internalState.batteryCharging) << internalState.maxTemperature;
  }
}
