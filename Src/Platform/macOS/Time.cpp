#include "Platform/Time.h"
#include "Platform/BHAssert.h"
#include <pthread.h>
#include <mach/mach_init.h>
#include <mach/thread_act.h>
#include <mach/mach_time.h>

unsigned Time::getRealSystemTime()
{
  static mach_timebase_info_data_t info = {0, 0};
  if(info.denom == 0)
    mach_timebase_info(&info);
  const unsigned long long time = mach_absolute_time() * info.numer / info.denom / 1000000;
  if(!base)
    base = time - 100000; // avoid time == 0, because it is often used as a marker
  return static_cast<unsigned>(time - base);
}

unsigned long long Time::getCurrentThreadTime()
{
  thread_basic_info_data_t tbid;
  mach_msg_type_number_t count = THREAD_BASIC_INFO_COUNT;
  VERIFY(!thread_info(pthread_mach_thread_np(pthread_self()), THREAD_BASIC_INFO, reinterpret_cast<thread_info_t>(&tbid), &count));
  const unsigned long long time = (tbid.user_time.seconds + tbid.system_time.seconds) * 1000000ll +
                                  tbid.user_time.microseconds + tbid.system_time.microseconds;
  if(!threadTimebase)
    threadTimebase = time - 1000000ll;
  return time - threadTimebase;
}
