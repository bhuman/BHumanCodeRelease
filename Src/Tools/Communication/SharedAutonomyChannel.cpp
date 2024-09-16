/**
 * @file Tools/Communication/SharedAutonomyChannel.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Thomas Röfer
 */

#include "SharedAutonomyChannel.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"

#ifdef WINDOWS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2ipdef.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

void SharedAutonomyChannel::startLocal(int port, unsigned localId)
{
  ASSERT(!this->port);
  this->port = port;
  this->localId = localId;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(false));
  std::string group = SystemCall::getHostAddr();
  group = "239" + group.substr(group.find('.'));
  VERIFY(socket.bind("0.0.0.0", port));
  VERIFY(socket.setTTL(0)); //keep packets off the network. non-standard(?), may work.
  VERIFY(socket.joinMulticast(group.c_str()));
  VERIFY(socket.setTarget(group.c_str(), port));
  socket.setLoopback(true);
}

void SharedAutonomyChannel::start(int port, const std::string& target)
{
  ASSERT(!this->port);
  this->port = port;
  this->target = target;

  socket.setBlocking(false);
  if(target.ends_with(".255"))
    VERIFY(socket.setBroadcast(true));
  VERIFY(socket.bind("0.0.0.0", port));
  socket.setLoopback(false);
  if(target != sendBack)
    socket.setTarget(target.c_str(), port);
}

void SharedAutonomyChannel::send(const char* data, int length)
{
  if(port)
    socket.write(data, length);
}

int SharedAutonomyChannel::receive(char* data, int length)
{
  if(!port)
    return -2; // not started yet

  unsigned remoteIp = 0;
  const int size = localId ? socket.readLocal(data, length)
                           : socket.read(data, length, remoteIp);
  if(!localId && size > 0 && target == sendBack)
  {
    unsigned ip = htonl(remoteIp);
    char addressBuffer[INET_ADDRSTRLEN];
    VERIFY(inet_ntop(AF_INET, &ip, addressBuffer, INET_ADDRSTRLEN) == addressBuffer);
    socket.setTarget(addressBuffer, port);
  }
  return size;
}
