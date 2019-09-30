/**
 * @file Platform/SystemCall.cpp
 *
 * Implementation of the system calls that are equal on all platforms.
 */

#include "SystemCall.h"
#include "BHAssert.h"

#ifdef WINDOWS
#include <Windows.h>
#else
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <cstring>
#endif
#include <string>

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
