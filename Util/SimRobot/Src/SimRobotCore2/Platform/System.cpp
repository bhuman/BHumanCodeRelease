/**
* @file System.cpp
* Implementation of class System
*/

#ifdef WINDOWS
#include <Windows.h>
#elif defined MACOS
#include <mach/mach_time.h>
#include <unistd.h>
#else
#include <ctime>
#include <unistd.h>
#endif

#include "System.h"

unsigned int System::getTime()
{
#ifdef WINDOWS
  return GetTickCount();
#elif defined MACOS
  static mach_timebase_info_data_t info = {0, 0};
  if(info.denom == 0)
    mach_timebase_info(&info);
  return static_cast<unsigned>(mach_absolute_time() * (info.numer / info.denom) / 1000000);
#else
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<unsigned int>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000l);
#endif
}
