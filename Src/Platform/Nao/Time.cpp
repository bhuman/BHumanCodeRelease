#include "Platform/Time.h"
#include "Platform/BHAssert.h"

#include <ctime>

unsigned long long Time::base = 0;
unsigned long long Time::threadTimebase = 0;

unsigned Time::getCurrentSystemTime()
{
  return getRealSystemTime();
}

unsigned Time::getRealSystemTime()
{
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const unsigned long long time = ts.tv_sec * 1000ll + ts.tv_nsec / 1000000;
  if(!base)
    base = time - 100000; // avoid time == 0, because it is often used as a marker
  return static_cast<unsigned>(time - base);
}

unsigned long long Time::getCurrentThreadTime()
{
  timespec ts;

  VERIFY(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0);

  const unsigned long long time = ts.tv_sec * 1000000ll + ts.tv_nsec / 1000;
  if(!threadTimebase)
    threadTimebase = time - 100000 * 1000;
  return time - threadTimebase;
}
