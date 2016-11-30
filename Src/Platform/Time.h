#pragma once

class Time
{
private:
  static unsigned base; /**< An offset used to convert the time to the time provided by this class. */
  static unsigned long long threadTimebase; /**< An offset used to convert the thread time to the time provided by this class. */

public:
  /** returns the current system time in milliseconds*/
  static unsigned getCurrentSystemTime();

  /** returns the real system time in milliseconds (never the simulated one)*/
  static unsigned getRealSystemTime();

  /** returns an offset used to convert the time to the time provided by this class. */
  static unsigned getSystemTimeBase();

  /**
   * The function returns the thread cpu time of the calling thread in microseconds.
   * return thread cpu time of the calling thread
   */
  static unsigned long long getCurrentThreadTime();

  /** returns the time since aTime*/
  static int getTimeSince(unsigned aTime);

  /** returns the real time since aTime*/
  static int getRealTimeSince(unsigned aTime);
};

inline unsigned Time::getSystemTimeBase()
{
  if(!base)
    (void)getRealSystemTime();
  return base;
}

inline int Time::getTimeSince(unsigned aTime)
{
  return static_cast<int>(getCurrentSystemTime() - aTime);
}

inline int Time::getRealTimeSince(unsigned aTime)
{
  return static_cast<int>(getRealSystemTime() - aTime);
}