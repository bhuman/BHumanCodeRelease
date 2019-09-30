#include "Platform/Time.h"
#include "Platform/BHAssert.h"

#ifdef TARGET_SIM
#include "Controller/RoboCupCtrl.h"
#endif

#include <ctime>
#include <pthread.h>

unsigned long long Time::base = 0;
unsigned long long Time::threadTimebase = 0;

unsigned Time::getCurrentSystemTime()
{
#ifdef TARGET_SIM
  if(RoboCupCtrl::controller)
    return RoboCupCtrl::controller->getTime();
  else
#endif
    return getRealSystemTime();
}

unsigned Time::getRealSystemTime()
{
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts); // NTP might change CLOCK_REALTIME on desktop systems
  const unsigned long long time = ts.tv_sec * 1000ll + ts.tv_nsec / 1000000;
  if(!base)
    base = time - 100000; // avoid time == 0, because it is often used as a marker
  return static_cast<unsigned>(time - base);
}

unsigned long long Time::getCurrentThreadTime()
{
  clockid_t cid;
  timespec ts;

  VERIFY(pthread_getcpuclockid(pthread_self(), &cid) == 0);
  VERIFY(clock_gettime(cid, &ts) == 0);

  const unsigned long long time = ts.tv_sec * 1000000ll + ts.tv_nsec / 1000;
  if(!threadTimebase)
    threadTimebase = time - 1000000ll;
  return time - threadTimebase;
}
