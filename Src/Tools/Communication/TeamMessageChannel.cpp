/**
 * @file Tools/Communication/TeamMessageChannel.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 *
 * based on TeamHandler.cpp authored by
 * @author Thomas Röfer
 */

#include "TeamMessageChannel.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"

void TeamMessageChannel::startLocal(int port, unsigned localId)
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
  targetSet = true;
}

void TeamMessageChannel::start(int port)
{
  ASSERT(!this->port);
  this->port = port;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(true));
  VERIFY(socket.bind("0.0.0.0", port));
  socket.setLoopback(false);
  trySetWifiTarget();
}

void TeamMessageChannel::send()
{
  if(!port || (!targetSet && !(trySetWifiTarget(), targetSet)))
    return;

  socket.write(reinterpret_cast<char*>(out.data), out.length);
}

bool TeamMessageChannel::receive()
{
  if(!port)
    return false; // not started yet

  int size;
  unsigned remoteIp = 0;

  size = localId ? socket.readLocal(reinterpret_cast<char*>(in.data), sizeof(in.data) + 1)
                 : socket.read(reinterpret_cast<char*>(in.data), sizeof(in.data) + 1, remoteIp);
  if(size < 1 || static_cast<size_t>(size) > sizeof(in.data))
    return false;
  else
    in.length = static_cast<uint8_t>(size);

  return true;
}

void TeamMessageChannel::trySetWifiTarget()
{
  std::string bcastAddr = UdpComm::getWifiBroadcastAddress();
  if(!bcastAddr.empty())
  {
    socket.setTarget(bcastAddr.c_str(), port);
    targetSet = true;
  }
}
