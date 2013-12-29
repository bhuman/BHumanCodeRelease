/**
* @file Controller/Representations/TimeInfo.cpp
*
* Implementation of class TimeInfo
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "TimeInfo.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Platform/SystemCall.h"
#include <cfloat>
#include "Tools/Debugging/Asserts.h"
#include <iostream>

using namespace std;

TimeInfo::TimeInfo(const std::string& name) : processName(name)
{
  reset();
}

void TimeInfo::reset()
{
  infos.clear();
  lastFrameNo = 0;
}

void TimeInfo::reset(Info& info)
{
  //FIXME
}

bool TimeInfo::handleMessage(InMessage& message)
{
  if(message.getMessageID() == idStopwatch)
  {
    timeStamp = SystemCall::getCurrentSystemTime();
    //first get the names of some of the stopwatches (usually we get 3 names per frame)
    unsigned short nameCount;
    message.bin >> nameCount;

    for(int i = 0; i < nameCount; ++i)
    {
      string watchName;
      unsigned short watchId;
      message.bin >> watchId;
      message.bin >> watchName;
      if(names.find(watchId) == names.end()) //new name
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
      infos[watchId].add(static_cast<float>(time));
    }
    unsigned processStartTime;
    message.bin >> processStartTime;
    unsigned frameNo;
    message.bin >> frameNo;

    int diff = frameNo - lastFrameNo;
    //sometimes we do not get data every frame. Compensate by assuming that the missing frames have
    // the same timing as the last one
    for(int i = 0; i < diff; ++i)
    {
      processDeltas.add(static_cast<float>(processStartTime - lastStartTime) / static_cast<float>(diff));
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
  avgTime = info.getAverage() / 1000.0f;
  minTime = info.getMinimum() / 1000.0f;
  maxTime = info.getMaximum() / 1000.0f;
}

void TimeInfo::getProcessStatistics(float& outAvgFreq) const
{
  outAvgFreq = 1000.0f / processDeltas.getAverage();
}

std::string TimeInfo::getName(unsigned short watchId) const
{
  if(names.find(watchId) == names.end())
  {
    return "unknown";
  }
  else
  {
    return names.at(watchId);
  }
}
