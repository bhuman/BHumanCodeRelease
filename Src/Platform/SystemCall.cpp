/**
 * @file Platform/SystemCall.cpp
 *
 * Implementation of the system calls that are equal on all platforms.
 */

#include "SystemCall.h"
#include "BHAssert.h"
#ifdef LINUX
#include "Linux/SoundPlayer.h"
#elif defined MACOS
#include "macOS/SoundPlayer.h"
#else
#include "Windows/SoundPlayer.h"
#endif

#ifdef WINDOWS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <Windows.h>
#else
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <cstring>
#endif
#include <string>

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
    static char buf[100];
    hostent* hostAddr = static_cast<hostent*>(gethostbyname(getHostName()));
    if(hostAddr && *hostAddr->h_addr_list)
      strcpy(buf, inet_ntoa(*reinterpret_cast<in_addr*>(*hostAddr->h_addr_list)));
    else
    {
      hostAddr = static_cast<hostent*>(gethostbyname((std::string(getHostName()) + ".local").c_str()));
      if(hostAddr && *hostAddr->h_addr_list)
        strcpy(buf, inet_ntoa(*reinterpret_cast<in_addr*>(*hostAddr->h_addr_list)));
      else
        strcpy(buf, "127.0.0.1");
    }
    hostaddr = buf;
  }
  return hostaddr;
}

void SystemCall::mute(bool isMuted)
{
  SystemCall::isMuted = isMuted;
}

int SystemCall::playSound(const char* name)
{
#ifdef TARGET_ROBOT
  fprintf(stderr, "Playing %s\n", name);
#endif
  return isMuted ? 0 : SoundPlayer::play(name);
}

int SystemCall::say(const char* text, float stretchFactor)
{
#ifdef TARGET_ROBOT
  fprintf(stderr, "Saying %s\n", text);
#endif
#ifdef LINUX
  return isMuted ? 0 : SoundPlayer::say(text, stretchFactor);
#else
  static_cast<void>(stretchFactor);
  return isMuted ? 0 : SoundPlayer::say(text);
#endif
}

bool SystemCall::soundIsPlaying()
{
  return SoundPlayer::isPlaying();
}
