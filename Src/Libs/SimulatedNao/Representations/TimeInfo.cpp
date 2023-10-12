/**
 * @file Representations/TimeInfo.cpp
 *
 * Implementation of class TimeInfo
 *
 * @author Thomas Röfer
 */

#include "TimeInfo.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"

void TimeInfo::reset()
{
  infos.clear();
  lastFrameNo = 0;
  lastStartTime = 0;
}

bool TimeInfo::handleMessage(MessageQueue::Message message, bool justReadNames)
{
  if(message.id() == idStopwatch)
  {
    auto stream = message.bin();
    timestamp = Time::getCurrentSystemTime();
    //first get the names of some of the stopwatches (usually we get 3 names per frame)
    unsigned short nameCount;
    stream >> nameCount;

    for(int i = 0; i < nameCount; ++i)
    {
      std::string watchName;
      unsigned short watchId;
      stream >> watchId;
      stream >> watchName;
      auto j = names.find(watchId);
      if(j == names.end() || j->second != watchName) //new or different name
      {
        names[watchId] = watchName;
        infos[watchId] = Info();
      }
    }

    //now get timing data
    unsigned short dataCount;
    stream >> dataCount;

    for(int i = 0; i < dataCount; ++i)
    {
      unsigned short watchId;
      unsigned time;
      stream >> watchId;
      stream >> time;
      if(!justReadNames)
        infos[watchId].push_front(static_cast<float>(time));
      infos[watchId].timestamp = Time::getCurrentSystemTime();
    }

    if(justReadNames)
      return true;

    unsigned threadStartTime;
    stream >> threadStartTime;
    unsigned frameNo;
    stream >> frameNo;

    int diff = frameNo - lastFrameNo;
    //sometimes we do not get data every frame. Compensate by assuming that the missing frames have
    // the same timing as the last one
    if(lastFrameNo && diff < static_cast<int>(threadDeltas.capacity()))
    {
      for(int i = 0; i < diff; ++i)
        threadDeltas.push_front(static_cast<float>(threadStartTime - lastStartTime) / static_cast<float>(diff));
    }

    lastFrameNo = frameNo;
    lastStartTime = threadStartTime;
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

void TimeInfo::getThreadStatistics(float& outAvgFreq, float& outMin, float& outMax) const
{
  outAvgFreq = threadDeltas.sum() != 0.f ? 1000.0f / threadDeltas.average() : 0.f;
  outMin = threadDeltas.minimum();
  outMax = threadDeltas.maximum();
}

std::string TimeInfo::getName(unsigned short watchId) const
{
  if(names.find(watchId) == names.end())
    return "unknown";
  else
    return names.at(watchId);
}
