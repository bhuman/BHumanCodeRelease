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
      theInternalState.maxTemperature = 0;
      for(float temperature : {theSystemSensorData.cpuTemperature, theSystemSensorData.gpuTemperature})
        if(temperature != SensorData::off && temperature > theInternalState.maxTemperature)
          theInternalState.maxTemperature = temperature;
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
      stream << internalState.batteryLevel << static_cast<int>(internalState.maxTemperature);
  }
}
