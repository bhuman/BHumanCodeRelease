/**
* @file Controller/Representations/TimeInfo.h
*
* Declaration of class TimeInfo
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include <string>
#include <unordered_map>
#include "Tools/RingBufferWithSum.h"

class InMessage;

/**
* @class TimeInfo
*
* A class to represent information about the timing of modules.
* There should be one TimeInfo per process.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author <a href="mailto:arneboe@tzi.de">Arne Böckmann</a>
*/
class TimeInfo
{
private:
  enum {ringBufferSize = 100};
public:
  std::string processName;
  typedef RingBufferWithSum<float, ringBufferSize> Info;
  typedef std::unordered_map<unsigned short, Info> Infos;
  Infos infos;
  unsigned int timeStamp; /**< The time stamp of the last change. */

  /**
  * Default constructor.
  */
  TimeInfo() = default;

  /**
  * Constructor.
  * @param name The name of the process the timings of which are stored
  *             in this object.
  */
  TimeInfo(const std::string& name);

  /**
  * The function handles a stop watch message.
  * @param message The message.
  * @return Was it a stop watch message?
  */
  bool handleMessage(InMessage& message);

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

  /**Returns the frequency of the process attached to this time info.
   */
  void getProcessStatistics(float& outAvgFreq) const;

  /**returns the name of the stopwatch with id watchId*/
  std::string getName(unsigned short watchId) const;
private:
  std::unordered_map<unsigned short, std::string> names;
  unsigned lastFrameNo; /**< frame number of the last received frame */
  unsigned lastStartTime; /**< The start time of the frame before this one */
  Info processDeltas; /**< contains the deltas between the recent process start times. Is used to calculate the frequency */
};
