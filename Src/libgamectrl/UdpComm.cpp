/**
 * @file UdpComm.cpp
 * Implements a wrapper for a UDP socket.
 * @author Armin Burchardt
 */

#include "UdpComm.h"

#include <iostream>
#include <cassert>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <cstring>
#include <net/if.h>
#include <ifaddrs.h>
#include <string>

UdpComm::UdpComm()
{
  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  target = (struct sockaddr*) (new struct sockaddr_in);

  assert(sock != -1);
}

UdpComm::~UdpComm()
{
  close(sock);
  delete (struct sockaddr_in*) target;
}

bool UdpComm::resolve(const char* addrStr, int port, struct sockaddr_in* addr)
{
  memset(addr, 0, sizeof(struct sockaddr_in));
  addr->sin_family = AF_INET;
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
  addr->sin_port = htons(port);
#ifdef __clang__
#pragma clang diagnostic pop
#endif
  if(inet_pton(AF_INET, addrStr, &(addr->sin_addr.s_addr)) != 1)
  {
    std::cerr << addrStr << " is not a valid dotted ipv4 address" << std::endl;
    return false;
  }

  return true;
}

bool UdpComm::setTarget(const char* addrStr, int port)
{
  struct sockaddr_in* addr = (struct sockaddr_in*) target;
  return resolve(addrStr, port, addr);
}

bool UdpComm::setBlocking(bool block)
{
  if(block)
    return fcntl(sock, F_SETFL, 0) != -1;
  else
    return fcntl(sock, F_SETFL, O_NONBLOCK) != -1;
}

bool UdpComm::setLoopback(bool yesno)
{
  char val = yesno ? 1 : 0;
  if(setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP, &val, sizeof(char)) < 0)
  {
    std::cerr << "could not set ip_multicast_loop to " << val << std::endl;
    return false;
  }
  return true;
}

bool UdpComm::setBroadcast(bool enable)
{
  int yes = enable ? 1 : 0;
  if(setsockopt(sock, SOL_SOCKET, SO_BROADCAST,
                (const char*) &yes, sizeof(yes)) == 0)
    return true;
  else
  {
    std::cerr << "UdpComm::setBroadcast() failed: " << strerror(errno) << std::endl;
    return false;
  }
}

bool UdpComm::bind(const char* addr_str, int port)
{
  static const int yes = 1;
  struct sockaddr_in addr;
  addr.sin_addr.s_addr = INADDR_ANY;
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
  addr.sin_port = htons(port);
#ifdef __clang__
#pragma clang diagnostic pop
#endif
  addr.sin_family = AF_INET;

  if(inet_pton(AF_INET, addr_str, &(addr.sin_addr)) <= 0)
  {
    std::cerr << "UdpComm::bind() failed: invalid address " << addr_str << std::endl;
    return false;
  }

#ifdef SO_REUSEADDR
  if(-1 == setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes)))
    std::cerr << "UdpComm: could not set SO_REUSEADDR" << std::endl;
#endif
#ifdef SO_REUSEPORT
  if(-1 == setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (const char*)&yes, sizeof(yes)))
    std::cerr << "UdpComm: could not set SO_REUSEPORT" << std::endl;
#endif
  if(-1 == ::bind(sock, (struct sockaddr*)&addr, sizeof(struct sockaddr_in)))
  {
    std::cout << "UdpComm::bind() failed: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

int UdpComm::read(char* data, int len)
{
  return ::recv(sock, data, len, 0);
}

bool UdpComm::write(const char* data, const int len)
{
  return ::sendto(sock, data, len, 0,
                  target, sizeof(struct sockaddr_in)) == len;
}

const char* UdpComm::getWifiBroadcastAddress()
{
  struct ifaddrs* ifAddrStruct = NULL;
  struct ifaddrs* ifa = NULL;

  //determine ip address
  getifaddrs(&ifAddrStruct);
  for(ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
  {
       // manpage getifaddrs    // check it is IP4
    if(ifa->ifa_addr != NULL && ifa->ifa_addr->sa_family == AF_INET)
    {
      std::string interfaceName(ifa->ifa_name);
      if(interfaceName.find("wlan") != std::string::npos)
      {
        in_addr_t mask = ((struct sockaddr_in *) ifa->ifa_netmask)->sin_addr.s_addr;
        in_addr_t addr = ((struct sockaddr_in *) ifa->ifa_addr)->sin_addr.s_addr;
        in_addr_t bcastAddr = ~mask | addr;

        struct in_addr bcast_addr;
        bcast_addr.s_addr = bcastAddr;
        static char buffer[INET_ADDRSTRLEN];
        inet_ntop(AF_INET,
                  &bcast_addr,
                  buffer,
                  INET_ADDRSTRLEN);
        return buffer;
      }
    }
  }
  return "255.255.255.255";
}
