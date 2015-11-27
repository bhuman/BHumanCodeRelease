/**
* @file  Platform/SimRobotQt/SystemCall.cpp
*
* Implementation of system calls and access to thread local storage.
* Only for use inside the simulator.
*/

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include "SystemCall.h"
#include "Platform/File.h"
#ifndef TARGET_TOOL
#  include "Controller/ConsoleRoboCupCtrl.h"
#  ifdef WINDOWS
#    include "Platform/Windows/SoundPlayer.h"
#  elif defined(OSX)
#    include "Platform/OSX/SoundPlayer.h"
#  else
#    include "Platform/Linux/SoundPlayer.h"
#  endif
#else
#  ifndef WINDOWS
#    ifdef LINUX
#      include <pthread.h>
#    endif
#    include <unistd.h>
#  else
#    define NOMINMAX
#    include <Windows.h>
#  endif
#  include <cstring>
#  include "Platform/BHAssert.h"
#endif

#ifndef WINDOWS
#  ifdef OSX
#    include <mach/mach_init.h>
#    include <mach/thread_act.h>
#    include <mach/mach_time.h>
#    include <sys/param.h>
#    include <sys/mount.h>
#  else
#    include <sys/sysinfo.h>
#    include <sys/statvfs.h>
#  endif
#  include <ctime>
#  include <netdb.h>
#  include <arpa/inet.h>
#endif

unsigned SystemCall::getCurrentSystemTime()
{
#ifndef TARGET_TOOL
  if(RoboCupCtrl::controller)
    return RoboCupCtrl::controller->getTime();
  else
#endif
    return getRealSystemTime();
}

unsigned SystemCall::getRealSystemTime()
{
#ifdef WINDOWS
  unsigned time = timeGetTime();
#elif defined(OSX)
  static mach_timebase_info_data_t info = {0, 0};
  if(info.denom == 0)
    mach_timebase_info(&info);
  unsigned int time = unsigned(mach_absolute_time() * (info.numer / info.denom) / 1000000);
#else
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  unsigned int time = (unsigned int)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000l);
#endif
  static unsigned base = 0;
  if(!base)
    base = time - 10000; // avoid time == 0, because it is often used as a marker
  return time - base;
}

unsigned long long SystemCall::getCurrentThreadTime()
{
#if defined(WINDOWS)
  static LARGE_INTEGER frequency = { 0 };
  if (frequency.QuadPart == 0)
  {
    QueryPerformanceFrequency(&frequency);
  }
  LARGE_INTEGER timeLL;
  QueryPerformanceCounter(&timeLL);
  return static_cast<unsigned long long>(timeLL.QuadPart * 1000000 / frequency.QuadPart);
#elif defined(OSX)
  thread_basic_info_data_t tbid;
  mach_msg_type_number_t count = THREAD_BASIC_INFO_COUNT;
  VERIFY(!thread_info(mach_thread_self(), THREAD_BASIC_INFO, (thread_info_t) &tbid, &count));
  return (tbid.user_time.seconds + tbid.system_time.seconds) * 1000000ll +
         tbid.user_time.microseconds + tbid.system_time.microseconds;
#else
  clockid_t cid;
  struct timespec ts;

  VERIFY(pthread_getcpuclockid(pthread_self(), &cid) == 0);
  VERIFY(clock_gettime(cid, &ts) == 0);

  unsigned long long time = ts.tv_sec * 1000000ll + ts.tv_nsec / 1000;

  static unsigned long long base = 0;
  if(!base)
    base = time - 1000000;
  return time - base;
#endif
}

const char* SystemCall::getHostName()
{
  static const char* hostname = 0;
  if(!hostname)
  {
    static char buf[100] = {0};
    VERIFY(!gethostname(buf, sizeof(buf)));
    hostname = buf;
  }
  return hostname;
}

const char* SystemCall::getHostAddr()
{
  static const char* hostaddr = 0;
  if(!hostaddr)
  {
    static char buf[100];
    hostent* hostAddr = (hostent*) gethostbyname(getHostName());
    if(hostAddr && *hostAddr->h_addr_list)
      strcpy(buf, inet_ntoa(*(in_addr*) *hostAddr->h_addr_list));
    else
      strcpy(buf, "127.0.0.1");
    hostaddr = buf;
  }
  return hostaddr;
}

SystemCall::Mode SystemCall::getMode()
{
#ifndef TARGET_TOOL
  if(RoboCupCtrl::controller)
    return ((ConsoleRoboCupCtrl*)RoboCupCtrl::controller)->getMode();
  else
#endif
    return teamRobot;
}

void SystemCall::sleep(unsigned int ms)
{
#ifdef WINDOWS
  Sleep(ms);
#else
  usleep(ms * 1000);
#endif
}

void SystemCall::getLoad(float& mem, float load[3])
{
#ifdef WINDOWS
  load[0] = load[1] = load[2] = -1.f; //Not implemented yet
  MEMORYSTATUS memStat;
  memStat.dwLength = sizeof(MEMORYSTATUS);
  GlobalMemoryStatus(&memStat);
  mem = float(memStat.dwMemoryLoad) / 100.f;
#elif defined(OSX) // FIXME
  mem = -1.f;
  load[0] = load[1] = load[2] = -1.f;
#else
  struct sysinfo info;
  if(sysinfo(&info) == -1)
    load[0] = load[1] = load[2] = mem = -1.f;
  else
  {
    load[0] = float(info.loads[0]) / 65536.f;
    load[1] = float(info.loads[1]) / 65536.f;
    load[2] = float(info.loads[2]) / 65536.f;
    mem = float(info.totalram - info.freeram) / float(info.totalram);
  }
#endif
}

unsigned long long SystemCall::getFreeDiskSpace(const char* path)
{
  std::string fullPath = File::isAbsolute(path) ? path : std::string(File::getBHDir()) + "/Config/" + path;
#ifdef WINDOWS
  for(std::string::size_type i = 0; i < fullPath.size(); ++i)
    if(fullPath[i] == '/')
      fullPath[i] = '\\';

  // Free space can only be determined for a directory
  DWORD attr = GetFileAttributes(fullPath.c_str());
  if(attr == 0xffffffff || !(attr & FILE_ATTRIBUTE_DIRECTORY))
  {
    std::string::size_type p = fullPath.rfind('\\');
    if(p != std::string::npos)
      fullPath = fullPath.substr(0, p + 1);
    ASSERT(GetFileAttributes(fullPath.c_str()) & FILE_ATTRIBUTE_DIRECTORY);
  }

  // UNC only works for roots
  if(fullPath.size() > 2 && fullPath[0] == '\\' && fullPath[1] == '\\')
  {
    std::string::size_type p = fullPath.find('\\', 2);
    p = fullPath.find('\\', p + 1);
    fullPath = fullPath.substr(0, p);
  }

  // UNC requires a trailing backslash
  if(!fullPath.empty() && fullPath.back() != '\\')
    fullPath += '\\';

  ULARGE_INTEGER freeBytesAvailable;
  if(GetDiskFreeSpaceEx(fullPath.c_str(), &freeBytesAvailable, nullptr, nullptr))
    return freeBytesAvailable.QuadPart;
  else
    return 0;
#elif defined(OSX)
  struct statfs data;
  if(!statfs(fullPath.c_str(), &data))
    return data.f_bavail * data.f_bsize;
  else
    return 0;
#else
  struct statvfs data;
  if(!statvfs(fullPath.c_str(), &data))
    return data.f_bavail * data.f_bsize;
  else
    return 0;
#endif
}

void* SystemCall::alignedMalloc(size_t size, size_t alignment)
{
#ifdef WINDOWS
  return _aligned_malloc(size, alignment);
#else
  void* ptr;
  if(!posix_memalign(&ptr, alignment, size))
  {
    return ptr;
  }
  else
  {
    return nullptr;
  }
#endif
}

void SystemCall::alignedFree(void* ptr)
{
#ifdef WINDOWS
  _aligned_free(ptr);
#else
  free(ptr);
#endif
}

#ifdef TARGET_SIM
int SystemCall::playSound(const char* name)
{
  return SoundPlayer::play(name);
}
#endif
