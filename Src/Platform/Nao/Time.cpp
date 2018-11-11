#include "Platform/Time.h"
#include "Platform/BHAssert.h"

#include <ctime>

unsigned Time::base = 0;
unsigned long long Time::threadTimebase = 0;

unsigned Time::getCurrentSystemTime()
{
  return getRealSystemTime();
}

unsigned Time::getRealSystemTime()
{
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts); // video4linux timestamps use this clock
  const unsigned int time = (unsigned int)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000l);
  if(!base)
    base = time - 100000; // avoid time == 0, because it is often used as a marker
  return time - base;
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
