/**
 * File:   TimingManager.cpp
 * Author: arne
 *
 * Created on May 9, 2013, 1:37 PM
 */

#include "TimingManager.h"
#include <unordered_map>
#include <vector>
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Debugging.h"
#include "Tools/MessageQueue/MessageQueue.h"

using namespace std;

struct TimingManager::Pimpl
{
  /**
   * NOTE: the hash maps only work because the compiler uses a string table and
   *       allocates only one address for all const string literals with the same value.
   *       If this ever changes you have to replace the key with std::string
   */

  /**
   * Key: name of the timer
   * value: If timer has been started but not stopped, yet: the start time.
   *        Else: the time between start and stop.
   */
  unordered_map<const char*, unsigned long long> timing;
  unordered_map<const char*, unsigned short> idTable; /**< Key: name of the stopwatch. Value: the id that is used when sending timing data over the network */
  unsigned currentThreadStartTime = 0; /**< Timestamp of the current thread iteration */
  unsigned frameNo = 0; /**<  Number of the current frame*/
  vector<const char*> watchNames; /**< Contains the names of the stopwatches */
  MessageQueue data; /**< Contains the timing data in streamable format inbetween frames */
  bool threadRunning = false; /**< Is a thread iteration running right now? */
  bool dataPrepared = false; /**< True if data hs already been prepared this frame */
  int watchNameIndex = 0; /**< Every frame a few watch names are transmitted. This is the index of the watchname that is to be transmitted next */
};

TimingManager::TimingManager() : prvt(new TimingManager::Pimpl)
{
  prvt->data.setSize(500000);
}

TimingManager::~TimingManager()
{
  delete prvt;
}

void TimingManager::startTiming(const char* identifier)
{
  auto timing = prvt->timing.find(identifier);
  if(timing == prvt->timing.end())
  {
    //create new entry
    prvt->watchNames.push_back(identifier);
    prvt->idTable[identifier] = static_cast<unsigned short>(prvt->idTable.size()); //NOTE: this assumes that an unsigned short will always be big big enough to count the timers...
    timing = prvt->timing.insert(std::pair<const char*, unsigned long long>(identifier, 0)).first;
  }
  prvt->dataPrepared = false;
  timing->second = Time::getCurrentThreadTime() - timing->second; // accumulate measurements
}

unsigned TimingManager::stopTiming(const char* identifier)
{
  const unsigned long long stopTime = Time::getCurrentThreadTime();
  auto timing = prvt->timing.find(identifier);
  const unsigned diff = unsigned(stopTime - timing->second);
  timing->second = diff;
  return diff;
}

void TimingManager::signalThreadStart()
{
  prvt->currentThreadStartTime = Time::getCurrentSystemTime();
  prvt->frameNo++;
  prvt->threadRunning = true;
  prvt->data.clear();
  prvt->dataPrepared = false;
  for(const pair<const char* const, unsigned long long>& it : prvt->timing)
    prvt->timing[it.first] = 0;
}

void TimingManager::signalThreadStop()
{
  prvt->threadRunning = false;
}

MessageQueue& TimingManager::getData()
{
  ASSERT(!prvt->threadRunning);
  if(!prvt->dataPrepared)
  {
    prepareData();
    prvt->dataPrepared = true;
  }
  return prvt->data;
}

void TimingManager::prepareData()
{
  /** Protocol:
   * unsigned short : number of stopwatch names (usually 3)
   * for each stopwatch name:
   *  unsigned short : id of the stopwatch
   *  string         : name of the stopwatch
   *
   * unsigned short : Total number of stopwatches
   * for each stopwatch:
   *  short : id of the stopwatch  (depending on where you start in the stream you might not know the name of this watch, yet)
   *  unsigned : time in microseconds
   *
   * unsigned : timestamp at which the last iteration started.
   * unsigned : frame number of the current frame
   */
  OutBinaryMessage& out = prvt->data.out.bin;

  // every frame we send 3 watch names
  out << static_cast<unsigned short>(3); //number of names to follow
  for(int i = 0; i < 3; ++i, prvt->watchNameIndex = (prvt->watchNameIndex + 1) % prvt->watchNames.size())
  {
    const char* watchName = prvt->watchNames[prvt->watchNameIndex];
    out << prvt->idTable[watchName] << watchName;
  }

  // now write the data of all watches
  out << static_cast<unsigned short>(prvt->timing.size());
  for(const pair<const char* const, unsigned long long>& it : prvt->timing)
  {
    out << prvt->idTable[it.first];
    out << static_cast<unsigned>(it.second); // the cast is ok because the time between start and stop will never be bigger than an int...
  }
  out << prvt->currentThreadStartTime;
  out << prvt->frameNo;
  if(!prvt->data.out.finishMessage(idStopwatch))
    OUTPUT_WARNING("TimingManager: queue is full!!!");
}
