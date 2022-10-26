#include "Time.h"
#ifdef MACOS
#include "Platform/BHAssert.h"
#include <pthread.h>
#include <mach/mach_init.h>
#include <mach/thread_act.h>
#include <mach/mach_time.h>
#elif defined WINDOWS
#include <Windows.h>
#elif defined LINUX
#include "Platform/BHAssert.h"
#ifndef TARGET_ROBOT
#include <pthread.h>
#endif
#include <time.h>
#endif

unsigned long long Time::base = 0;
unsigned long long Time::threadTimebase = 0;

#ifndef TARGET_ROBOT

bool Time::isInitialized = false;
bool Time::isTimeSimulated = false;
int Time::simulatedTime = 0;

void Time::initialize()
{
  simulatedTime = 100000 - getRealSystemTime();
  isTimeSimulated = false;
  isInitialized = true;
}

void Time::deinitialize()
{
  isInitialized = false;
}

void Time::setSimulatedTime(bool on)
{
  if(isTimeSimulated != on)
  {
    if(on)
      simulatedTime += getRealSystemTime();
    else
      simulatedTime -= getRealSystemTime();
    isTimeSimulated = on;
  }
}

void Time::addSimulatedTime(int dt)
{
  if(isTimeSimulated)
    simulatedTime += dt;
}

#endif

unsigned Time::getCurrentSystemTime()
{
#ifndef TARGET_ROBOT
  if(isInitialized)
  {
    if(isTimeSimulated)
      return simulatedTime;
    else
      return unsigned(getRealSystemTime() + simulatedTime);
  }
  else
#endif
    return getRealSystemTime();
}

unsigned Time::getRealSystemTime()
{
#ifdef MACOS
  static mach_timebase_info_data_t info = {0, 0};
  if(info.denom == 0)
    mach_timebase_info(&info);
  const unsigned long long time = mach_absolute_time() * info.numer / info.denom / 1000000;
#elif defined WINDOWS
  const unsigned time = timeGetTime();
#elif defined LINUX
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts); // NTP might change CLOCK_REALTIME on desktop systems
  const unsigned long long time = ts.tv_sec * 1000ll + ts.tv_nsec / 1000000;
#endif
  if(!base)
    base = time - 100000; // avoid time == 0, because it is often used as a marker
  return static_cast<unsigned>(time - base);
}

unsigned long long Time::getCurrentThreadTime()
{
#ifdef MACOS
  thread_basic_info_data_t tbid;
  mach_msg_type_number_t count = THREAD_BASIC_INFO_COUNT;
  VERIFY(!thread_info(pthread_mach_thread_np(pthread_self()), THREAD_BASIC_INFO, reinterpret_cast<thread_info_t>(&tbid), &count));
  const unsigned long long time = (tbid.user_time.seconds + tbid.system_time.seconds) * 1000000ll +
                                  tbid.user_time.microseconds + tbid.system_time.microseconds;
#elif defined WINDOWS
  static LARGE_INTEGER frequency = { 0 };
  if(frequency.QuadPart == 0)
    QueryPerformanceFrequency(&frequency);
  LARGE_INTEGER timeLL;
  QueryPerformanceCounter(&timeLL);

  const unsigned long long time = static_cast<unsigned long long>(timeLL.QuadPart * 1000000ll / frequency.QuadPart);
#elif defined LINUX
  timespec ts;

#ifdef TARGET_ROBOT
  VERIFY(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0);
#else
  clockid_t cid;
  VERIFY(pthread_getcpuclockid(pthread_self(), &cid) == 0);
  VERIFY(clock_gettime(cid, &ts) == 0);
#endif

  const unsigned long long time = ts.tv_sec * 1000000ll + ts.tv_nsec / 1000;
#endif
  if(!threadTimebase)
    threadTimebase = time - 1000000ll;
  return time - threadTimebase;
}
