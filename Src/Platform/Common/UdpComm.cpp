/**
 * @file Platform/Common/UdpComm.cpp
 * Wrapper for an udp socket.
 * \author Armin
 */

#include "UdpComm.h"

#include <iostream>
#ifdef WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
#else
#  include <cerrno>
#  include <unistd.h>
#  include <fcntl.h>
#  include <sys/socket.h>
#  include <sys/types.h>
#  include <sys/ioctl.h>
#  include <net/if.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <netdb.h>
#  include <cstring>
#  include <net/if.h>
#  include <ifaddrs.h>
#endif

#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"

UdpComm::UdpComm()
{
  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  target = (struct sockaddr*)(new struct sockaddr_in);

  ASSERT(-1 != sock);
}

UdpComm::~UdpComm()
{
#ifdef WIN32
  closesocket(sock);
#else
  close(sock);
#endif

  delete (struct sockaddr_in*)target;
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
  if(1 != inet_pton(AF_INET, addrStr, &(addr->sin_addr.s_addr)))
  {
    std::cerr << addrStr << " is not a valid dotted ipv4 address" << std::endl;
    return false;
  }

  return true;
}


bool UdpComm::setTarget(const char* addrStr, int port)
{
  struct sockaddr_in* addr = (struct sockaddr_in*)target;
  return resolve(addrStr, port, addr);
}


bool UdpComm::setBlocking(bool block)
{
#ifdef WIN32
  int yes = block ? 0 : 1;
  if(ioctlsocket(sock, FIONBIO, (u_long*)&yes))
    return false;
  else
    return true;
#else
  bool result(false);
  if(block)
  {
    if(-1 != fcntl(sock, F_SETFL, 0))
      result = true;
  }
  else
  {
    if(-1 != fcntl(sock, F_SETFL, O_NONBLOCK))
      result = true;
  }
  return result;
#endif
}


bool UdpComm::setTTL(const char ttl)
{
  int r = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(unsigned char));
  if(r < 0)
  {
    std::cerr << "could not set TTL to " << ttl << std::endl;
    return false;
  }
  return true;
}


bool UdpComm::setLoopback(bool yesno)
{
  char val = yesno ? 1 : 0;
  int r = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP, &val, sizeof(char));
  if(r < 0)
  {
    std::cerr << "could not set ip_multicast_loop to " << val << std::endl;
    return false;
  }
  return true;
}


bool UdpComm::joinMulticast(const char* addrStr)
{
  struct sockaddr_in group;
  if(!resolve(addrStr, 0, &group))
    return false;

  //join multicast group for every interface
  if(IN_MULTICAST(ntohl(group.sin_addr.s_addr)))
  {
#ifndef WIN32
    struct ip_mreq mreq;
    struct ifconf ifc;
    struct ifreq* item;
    char buf[1024];

    ifc.ifc_len = sizeof(buf);
    ifc.ifc_buf = buf;
    if(ioctl(sock, SIOCGIFCONF, &ifc) < 0)
    {
      std::cerr << "cannot get interface list" << std::endl;
      return false;
    }
    else
    {
      bool could_join(false);
      for(unsigned int i = 0; i < ifc.ifc_len / sizeof(struct ifreq); i++)
      {
        item = &ifc.ifc_req[i];
        mreq.imr_multiaddr = group.sin_addr;
        mreq.imr_interface = ((struct sockaddr_in*)&item->ifr_addr)->sin_addr;
        if(0 == setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                           (void*)&mreq, sizeof(mreq)))
        {
          could_join = true;
        }
      }
      if(! could_join)
      {
        std::cerr << "join multicast group failed for interface" << std::endl;
        return false;
      }
    }
#else
    char host[128];
    struct hostent* pHost;
    if(gethostname(host, sizeof(host)) < 0 || !(pHost = (struct hostent*)gethostbyname(host)))
    {
      std::cerr << "cannot get interface list" << std::endl;
      return false;
    }

    struct ip_mreq mreq;
    bool couldJoin(false);
    for(int i = 0; pHost->h_addr_list[i]; i++)
    {
      mreq.imr_multiaddr = group.sin_addr;
      mreq.imr_interface = *((struct in_addr*)pHost->h_addr_list[i]);
      if(setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) == 0)
        couldJoin = true;
    }
    if(!couldJoin)
    {
      std::cerr << "join multicast group failed for interface" << std::endl;
      return false;
    }
#endif
    return true;
  }
  else
    std::cerr << "not a multicast address" << std::endl;
  return false;
}


bool UdpComm::setBroadcast(bool enable)
{
  int yes = enable ? 1 : 0;
  if(0 == setsockopt(sock, SOL_SOCKET, SO_BROADCAST,
                     (const char*) &yes, sizeof(yes)))
  {
    return true;
  }
  else
  {
    std::cerr << "UdpComm::setBroadcast() failed: " << strerror(errno) << std::endl;
    return false;
  }
}

bool UdpComm::setRcvBufSize(unsigned int rcvbuf)
{
  if(0 == setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (char*) &rcvbuf, sizeof(rcvbuf)))
  {
    std::cerr << "multicast-socket: setsockopt for SO_RCVBUF failed: "
              << strerror(errno) << std::endl;
    return false;
  }

  int result;
  socklen_t result_len = sizeof(result);
  if(0 == getsockopt(sock, SOL_SOCKET, SO_RCVBUF, (char*) &result, &result_len))
  {
    std::cerr << "multicast-socket: receive buffer set to "
              << result << " Bytes." << std::endl;
    return true;
  }

  std::cerr << "multicast-socket: could not get sockopt SO_RCVBUF" << std::endl;
  return false;
}


bool UdpComm::bind(const char* addr_str, int port)
{
  static const int yes = 1;
  struct sockaddr_in addr;
  addr.sin_addr.s_addr = INADDR_ANY; //addr.sin_addr;
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
  addr.sin_port = htons(port);
#ifdef __clang__
#pragma clang diagnostic pop
#endif
  addr.sin_family = AF_INET;

#ifdef BHUMAN_USE_INET_ADDR
  addr.sin_addr.s_addr = inet_addr(addr_str);
#else
  int r = inet_pton(AF_INET, addr_str, &(addr.sin_addr));
  if(r <= 0)
  {
    std::cerr << "UdpComm::bind() failed: invalid address " << addr_str << std::endl;
    return false;
  }
#endif

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

int UdpComm::read(char* data, int len, unsigned int& ip)
{
  sockaddr_in senderAddr;
#ifdef WIN32
  int size = sizeof(senderAddr);
#else
  unsigned size = sizeof(senderAddr);
#endif
  int result = ::recvfrom(sock, data, len, 0, (sockaddr*) &senderAddr, &size);
  if(result <= 0)
    return result;
  ip = ntohl(senderAddr.sin_addr.s_addr);
  return result;
}

int UdpComm::read(char* data, int len)
{
  return ::recv(sock, data, len, 0);
}

int UdpComm::readLocal(char* data, int len)
{
  sockaddr_in senderAddr;
#ifdef WIN32
  int size = sizeof(senderAddr);
#else
  unsigned size = sizeof(senderAddr);
  bool found = false;
#endif
  int result = ::recvfrom(sock, data, len, 0, (sockaddr*) &senderAddr, &size);
  if(result <= 0)
    return result;
  else
  {
#ifndef WIN32
    struct ifaddrs* addrs, *ifac;

    if(getifaddrs(&addrs) < 0)
      return -1;

    for(ifac = addrs; !found && ifac != NULL; ifac = ifac->ifa_next)
    {
      if(ifac->ifa_flags & IFF_MULTICAST
         && ifac->ifa_addr
         && ifac->ifa_addr->sa_family == AF_INET)
      {
        if(((struct sockaddr_in*)ifac->ifa_addr)->sin_addr.s_addr == senderAddr.sin_addr.s_addr)
          found = true;
      }
    }

    freeifaddrs(addrs);

    // no, package comes from the outside -> ignore
    return found ? result : -1;
#else
    return result;
#endif
  }
}

bool UdpComm::write(const char* data, const int len)
{
  return ::sendto(sock, data, len, 0,
                  target, sizeof(struct sockaddr_in)) == len;
}

std::string UdpComm::getWifiBroadcastAddress()
{
  std::string wifiAddress("255.255.255.255");
  #ifdef TARGET_ROBOT
  char addressBuffer[INET_ADDRSTRLEN];

  struct ifaddrs * ifAddrStruct = NULL;
  struct ifaddrs * ifa =NULL;

  //determine ip address
  getifaddrs(&ifAddrStruct);
  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
  {
        // manpage getifaddrs    // check it is IP4
    if (ifa->ifa_addr != NULL && ifa->ifa_addr->sa_family==AF_INET)
    {
      std::string interfaceName(ifa->ifa_name);
      if (interfaceName.find("wlan") != std::string::npos)
      {
        in_addr_t mask = ((struct sockaddr_in *)ifa->ifa_netmask)->sin_addr.s_addr;
        in_addr_t addr = ((struct sockaddr_in *)ifa->ifa_addr)->sin_addr.s_addr;
        in_addr_t bcastAddr = ~mask | addr;

        struct in_addr bcast_addr;
        bcast_addr.s_addr = bcastAddr;

        inet_ntop(AF_INET,
                  &bcast_addr,
                  addressBuffer,
                  INET_ADDRSTRLEN);
        wifiAddress = std::string(&addressBuffer[0]);
        break;
      }
    }
  }
  #endif
  return wifiAddress;
}
