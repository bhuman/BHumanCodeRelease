/**
 * File:   TimingManager.h
 * Author: arne
 *
 * Created on May 9, 2013, 1:37 PM
 */

#pragma once

class MessageQueue;

/**
 * A class that keeps track of several stopwatches.
 */
class TimingManager final
{
public:
  /** Constructor. */
  TimingManager();

  /** Destructor. */
  ~TimingManager();

  /** Start the stopwatch for the specified identifier. */
  void startTiming(const char* identifier);

  /** Stops the stopwatch for the specified identifier and returns the time in us. */
  unsigned stopTiming(const char* identifier);

  /**
   * The TimingManager has a special stopwatch that is used to keep track
   * of the overall thread time.
   * You should call signalThreadStart at the beginning of every thread iteration.
   * It is used to calculate the frequency of the thread.
   */
  void signalThreadStart();

  /**
   * Returns a message queue that contains all timing data from this frame.
   * Call this method in between signalThreadStop() and signalThreadStart.
   */
  MessageQueue& getData();

private:
  /** Prepares timing data for streaming. */
  void prepareData();

  struct Pimpl;
  Pimpl* prvt;
};
