/**
 * @file Controller/Representations/TimeInfo.cpp
 *
 * Implementation of class TimeInfo
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "TimeInfo.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Platform/Time.h"
#include "Platform/BHAssert.h"
#include <iostream>

TimeInfo::TimeInfo(const std::string& name, int frameNoDivisor) : processName(name), frameNoDivisor(frameNoDivisor)
{
  reset();
}

void TimeInfo::reset()
{
  infos.clear();
  lastFrameNo = 0;
  lastStartTime = 0;
}

bool TimeInfo::handleMessage(InMessage& message, bool justReadNames)
{
  if(message.getMessageID() == idStopwatch)
  {
    timeStamp = Time::getCurrentSystemTime();
    //first get the names of some of the stopwatches (usually we get 3 names per frame)
    unsigned short nameCount;
    message.bin >> nameCount;

    for(int i = 0; i < nameCount; ++i)
    {
      std::string watchName;
      unsigned short watchId;
      message.bin >> watchId;
      message.bin >> watchName;
      auto j = names.find(watchId);
      if(j == names.end() || j->second != watchName) //new or different name
      {
        names[watchId] = watchName;
        infos[watchId] = Info();
      }
    }

    //now get timing data
    unsigned short dataCount;
    message.bin >> dataCount;

    for(int i = 0; i < dataCount; ++i)
    {
      unsigned short watchId;
      unsigned time;
      message.bin >> watchId;
      message.bin >> time;
      if(!justReadNames)
        infos[watchId].push_front(static_cast<float>(time));
      infos[watchId].timeStamp = Time::getCurrentSystemTime();
    }

    if(justReadNames)
      return true;

    unsigned processStartTime;
    message.bin >> processStartTime;
    unsigned frameNo;
    message.bin >> frameNo;
    frameNo /= frameNoDivisor;

    int diff = frameNo - lastFrameNo;
    //sometimes we do not get data every frame. Compensate by assuming that the missing frames have
    // the same timing as the last one
    if(lastFrameNo && diff < static_cast<int>(processDeltas.capacity()))
    {
      for(int i = 0; i < diff; ++i)
        processDeltas.push_front(static_cast<float>(processStartTime - lastStartTime) / static_cast<float>(diff));
    }

    lastFrameNo = frameNo;
    lastStartTime = processStartTime;
    return true;
  }
  else
    return false;
}

void TimeInfo::getStatistics(const Info& info, float& minTime, float& maxTime, float& avgTime) const
{
  avgTime = info.average() / 1000.0f;
  minTime = info.minimum() / 1000.0f;
  maxTime = info.maximum() / 1000.0f;
}

void TimeInfo::getProcessStatistics(float& outAvgFreq, float& outMin, float& outMax) const
{
  outAvgFreq = processDeltas.sum() != 0.f ? 1000.0f / processDeltas.average() : 0.f;
  outMin = processDeltas.minimum();
  outMax = processDeltas.maximum();
}

std::string TimeInfo::getName(unsigned short watchId) const
{
  if(names.find(watchId) == names.end())
    return "unknown";
  else
    return names.at(watchId);
}
