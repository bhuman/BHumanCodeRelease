/**
 * File:   TimingManager.h
 * Author: arne
 *
 * Created on May 9, 2013, 1:37 PM
 */

#pragma once

class Process;
class MessageQueue;

/**
 * A class that keeps track of several stopwatches.
 * It always belongs to exactly one process and should only be created/destroyed
 * by that process.
 * There should be exactly one TimingManager per process.
 */
class TimingManager
{
private:
  struct Pimpl;
  Pimpl* prvt;

  friend class Process;
  TimingManager(); // private so only Process can access it.
  ~TimingManager();

public:
  /** Start the stopwatch for the specified identifier. */
  void startTiming(const char* identifier);

  /** Stops the stopwatch for the specified identifier and returns the time in us. */
  unsigned stopTiming(const char* identifier);

  /**
   * The TimingManager has a special stopwatch that is used to keep track
   * of the overall process time.
   * You should call signalProcessStart at the beginning of every process iteration.
   * It is used to calculate the frequency of the process.
   */
  void signalProcessStart();

  /** Tells the TimingManager that the current process iteration is over. */
  void signalProcessStop();

  /**
   * Returns a message queue that contains all timing data from this frame.
   * Call this method in between signalProcessStop() and signalProcessStart.
   */
  MessageQueue& getData();

private:
  /** Prepares timing data for streaming. */
  void prepareData();
};
