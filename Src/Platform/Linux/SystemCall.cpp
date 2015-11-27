/**
* @file Platform/linux/SystemCall.cpp
*
* Implementation of system calls and access to thread local storage.
* Only for use on Linux.
*
* @author <A href=mailto:brunn@sim.informatik.tu-darmstadt.de>Ronnie Brunn</A>
* @author <A href=mailto:martin@martin-loetzsch.de>Martin Lötzsch</A>
* @author <A href=mailto:risler@sim.informatik.tu-darmstadt.de>Max Risler</A>
* @author <a href=mailto:dueffert@informatik.hu-berlin.de>Uwe Düffert</a>
*/

#include "SystemCall.h"
#include "Platform/File.h"
#include "SoundPlayer.h"
#include "BHAssert.h"
#include <sys/sysinfo.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <cstring>
#include <ctime>
#include <sys/statvfs.h>

unsigned SystemCall::base = 0;

unsigned SystemCall::getCurrentSystemTime()
{
  return getRealSystemTime();
}

unsigned SystemCall::getRealSystemTime()
{
  struct timespec ts;
#ifdef TARGET_ROBOT
  clock_gettime(CLOCK_REALTIME, &ts); // video4linux timestamps use this clock
#else
  clock_gettime(CLOCK_MONOTONIC, &ts); // NTP might change CLOCK_REALTIME on desktop systems
#endif
  unsigned int time = (unsigned int)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000l);
  if(!base)
    base = time - 10000; // avoid time == 0, because it is often used as a marker
  return time - base;
}

unsigned SystemCall::getSystemTimeBase()
{
  if(!base)
    (void) getRealSystemTime();
  return base;
}

unsigned long long SystemCall::getCurrentThreadTime()
{
  struct timespec ts;

  VERIFY(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0);

  unsigned long long time = ts.tv_sec * 1000000ll + ts.tv_nsec / 1000;

  static unsigned long long base = 0;
  if(!base)
    base = time - 10000 * 1000;
  return time - base;
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
#ifdef STATIC // Prevent warnings during static linking
  ASSERT(false); // should not be called
#else
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
#endif
  return hostaddr;
}

SystemCall::Mode SystemCall::getMode()
{
  return physicalRobot;
}

void SystemCall::sleep(unsigned ms)
{
  usleep(1000 * ms);
}

void SystemCall::getLoad(float& mem, float load[3])
{
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
}

unsigned long long SystemCall::getFreeDiskSpace(const char* path)
{
  std::string fullPath = File::isAbsolute(path) ? path : std::string(File::getBHDir()) + "/" + path;
  struct statvfs data;
  if(!statvfs(fullPath.c_str(), &data))
    return data.f_bfree * data.f_bsize;
  else
    return 0;
}

void* SystemCall::alignedMalloc(size_t size, size_t alignment)
{
  void* ptr;
  if(!posix_memalign(&ptr, alignment, size))
  {
    return ptr;
  }
  else
  {
    return nullptr;
  }
}

void SystemCall::alignedFree(void* ptr)
{
  free(ptr);
}

int SystemCall::playSound(const char* name)
{
#ifdef TARGET_ROBOT
  fprintf(stderr, "Playing %s\n", name);
#endif
  return SoundPlayer::play(name);
}
