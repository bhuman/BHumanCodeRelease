/**
 * @file Controller/Representations/TimeInfo.h
 *
 * Declaration of class TimeInfo
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/RingBufferWithSum.h"

#include <string>
#include <unordered_map>

class InMessage;

// Extended Ringbuffer with timestamp containing the last update of the timing info
class InfoWithTimestamp : public RingBufferWithSum<float, 100>
{
public:
  unsigned int timestamp = 0;
};

/**
 * @class TimeInfo
 *
 * A class to represent information about the timing of modules.
 * There should be one TimeInfo per thread.
 *
 * @author Thomas Röfer
 * @author <a href="mailto:arneboe@tzi.de">Arne Böckmann</a>
 */
class TimeInfo
{
public:
  using Info = InfoWithTimestamp;
  using Infos = std::unordered_map<unsigned short, Info>;

  std::string threadName;
  Infos infos;
  unsigned int timestamp; /**< The timestamp of the last change. */

private:
  std::unordered_map<unsigned short, std::string> names;
  unsigned lastFrameNo; /**< frame number of the last received frame */
  unsigned lastStartTime; /**< The start time of the frame before this one */
  Info threadDeltas; /**< contains the deltas between the recent thread start times. Is used to calculate the frequency */

public:
  TimeInfo() {reset();}

  /**
   * The function handles a stop watch message.
   * @param message The message.
   * @param justReadNames Only read stopwatch names. Do not update statistics.
   * @return Was it a stop watch message?
   */
  bool handleMessage(InMessage& message, bool justReadNames = false);

  /**
   * The function empties the time info object.
   */
  void reset();

  /**
   * The function returns statistics about a certain stop watch.
   * @param info Information on the stop watch to query.
   * @param minTime The shortest measurement is returned to this variable in ms.
   * @param maxTime The longest measurement is returned to this variable in ms.
   * @param avgTime The average measurement is returned to this variable in ms.
   */
  void getStatistics(const Info& info, float& outMinTime, float& outMaxTime, float& outAvgTime) const;

  /**
   * Returns the frequency of the process attached to this time info.
   */
  void getThreadStatistics(float& outAvgFreq, float& outMin, float& outMax) const;

  /**
   * Returns the name of the stopwatch with id watchId
   */
  std::string getName(unsigned short watchId) const;
};
