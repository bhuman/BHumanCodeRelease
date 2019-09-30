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
 * It always belongs to exactly one thread and should only be created/destroyed
 * by that thread.
 * There should be exactly one TimingManager per thread.
 */
class TimingManager
{
private:
  struct Pimpl;
  Pimpl* prvt;

  friend class ThreadFrame; /**< A thread is allowed to create the instance. */
  /**
   * Default constructor.
   * No other instance of this class is allowed except the one accessible via Global::getTimingManager.
   * Therefore the constructor is private.
   */
  TimingManager();
  ~TimingManager();

public:
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

  /** Tells the TimingManager that the current thread iteration is over. */
  void signalThreadStop();

  /**
   * Returns a message queue that contains all timing data from this frame.
   * Call this method in between signalThreadStop() and signalThreadStart.
   */
  MessageQueue& getData();

private:
  /** Prepares timing data for streaming. */
  void prepareData();
};
