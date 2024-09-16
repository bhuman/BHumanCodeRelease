/**
 * @file Platform/SystemCall.cpp
 *
 * Implementation of the system calls that are equal on all platforms.
 */

#include "SystemCall.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#ifdef LINUX
#include "Platform/Linux/SoundPlayer.h"
#elif defined MACOS
#include "Platform/macOS/SoundPlayer.h"
#else
#include "Platform/Windows/SoundPlayer.h"
#endif

#ifdef WINDOWS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2ipdef.h>
#include <ws2tcpip.h>
#else
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <cstring>
#ifdef LINUX
#include <sys/sysinfo.h>
#include <sys/statvfs.h>
#elif defined MACOS
#include <sys/mount.h>
#include <sys/param.h>
#endif
#endif
#include <string>

#ifdef TARGET_ROBOT
#include <cstdlib>
#endif

bool SystemCall::isMuted = false;

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
    static char buf[INET_ADDRSTRLEN];
    hostent* hostAddr = static_cast<hostent*>(gethostbyname(getHostName()));
    if(hostAddr && *hostAddr->h_addr_list)
      inet_ntop(AF_INET, *hostAddr->h_addr_list, buf, INET_ADDRSTRLEN);
    else
    {
      hostAddr = static_cast<hostent*>(gethostbyname((std::string(getHostName()) + ".local").c_str()));
      if(hostAddr && *hostAddr->h_addr_list)
        inet_ntop(AF_INET, *hostAddr->h_addr_list, buf, INET_ADDRSTRLEN);
      else
        strcpy(buf, "127.0.0.1");
    }
    hostaddr = buf;
  }
  return hostaddr;
}

void SystemCall::getLoad(float& mem, float load[3])
{
#ifdef LINUX
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
#elif defined WINDOWS
  load[0] = load[1] = load[2] = -1.f; //Not implemented yet
  MEMORYSTATUS memStat;
  memStat.dwLength = sizeof(MEMORYSTATUS);
  GlobalMemoryStatus(&memStat);
  mem = float(memStat.dwMemoryLoad) / 100.f;
#else
  mem = -1.f;
  load[0] = load[1] = load[2] = -1.f;
#endif
}

unsigned long long SystemCall::getFreeDiskSpace(const char* path)
{
  std::string fullPath = File::isAbsolute(path) ? path : std::string(File::getBHDir()) + "/Config/" + path;
#ifdef LINUX
  struct statvfs data;
  if(!statvfs(fullPath.c_str(), &data))
#ifdef TARGET_ROBOT
    return static_cast<unsigned long long>(data.f_bfree)
           * static_cast<unsigned long long>(data.f_bsize);
#else
    return data.f_bavail * data.f_bsize;
#endif
  else
    return 0;
#elif defined MACOS
  struct statfs data;
  if(!statfs(fullPath.c_str(), &data))
    return data.f_bavail * data.f_bsize;
  else
    return 0;
#elif defined WINDOWS
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
#endif
}

void SystemCall::mute(bool isMuted)
{
  SystemCall::isMuted = isMuted;
}

int SystemCall::playSound(const char* name, bool force)
{
#ifdef LINUX
  return SoundPlayer::play(name, isMuted && !force);
#else
  return isMuted && !force ? 0 : SoundPlayer::play(name);
#endif
}

int SystemCall::say(const char* text, bool force, float stretchFactor)
{
#ifdef LINUX
  return SoundPlayer::say(text, isMuted && !force, stretchFactor);
#else
  static_cast<void>(stretchFactor);
  return isMuted && !force  ? 0 : SoundPlayer::say(text);
#endif
}

bool SystemCall::soundIsPlaying()
{
  return SoundPlayer::isPlaying();
}

bool SystemCall::usbIsMounted()
{
#ifdef TARGET_ROBOT
  return std::system("mount | grep \"media/usb\" >/dev/null") == 0;
#else
  return false;
#endif
}
