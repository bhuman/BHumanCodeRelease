/**
* @file Tools/ProcessFramework/TeamHandler.cpp
* The file implements a class for team communication between robots.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#include "TeamHandler.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/InStreams.h"

TeamHandler::TeamHandler(MessageQueue& in, MessageQueue& out) :
  in(in),
  out(out),
  port(0), localId(0)
{
}

void TeamHandler::startLocal(int port, unsigned short localId)
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
}

void TeamHandler::start(int port, const char* subnet)
{
  ASSERT(!this->port);
  this->port = port;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(true));
  VERIFY(socket.bind("0.0.0.0", port));
  socket.setTarget(subnet, port);
  socket.setLoopback(false);
}

void TeamHandler::send()
{
  if(!port || out.isEmpty())
    return;

  const int teamCommHeaderSize = localId ? 8 : 6;
  char buffer[1400];

  OutBinarySize sizeStream;
  sizeStream << out;
  int size = sizeStream.getSize() + teamCommHeaderSize;
  if(size > int(sizeof(buffer)))
  {
    out.clear();
    return;
  }

  OutBinaryMemory memory(buffer + teamCommHeaderSize);
  memory << out;
  // TeamComm header (remote)
  // Byte |  0  1  2  3   |    4    5    |
  //      |   timestamp   | payload size |
  // TeamComm header (local)
  // Byte |  0  1  2  3   |    4    5    |   6  7   |
  //      |   timestamp   | payload size | local ID |
  ((unsigned int*)buffer)[0] = SystemCall::getCurrentSystemTime();
  ((unsigned short*)buffer)[2] = (unsigned short) size;
  if(localId)
  {
    ((unsigned short*)buffer)[3] = localId;
  }

  if(socket.write(buffer, size))
    out.clear();
}

void TeamHandler::receive()
{
  if(!port)
    return; // not started yet

  const int teamCommHeaderSize = localId ? 8 : 6;
  char buffer[1400];
  int size;
  unsigned int remoteIp = 0;

  do
  {
    size = localId ? socket.readLocal(buffer, sizeof(buffer)) : socket.read(buffer, sizeof(buffer), remoteIp);
    if(size >= teamCommHeaderSize && ((unsigned short*)buffer)[2] == size)
    {
      in.out.bin << (localId ? ((unsigned short*)buffer)[3] : remoteIp);
      in.out.bin << ((unsigned int*)buffer)[0]; // the send time stamp
      in.out.bin << SystemCall::getCurrentSystemTime(); // the receive time stamp
      in.out.bin << (unsigned short)size;
      in.out.finishMessage(idNTPHeader);

      InBinaryMemory memory(buffer + teamCommHeaderSize, size - teamCommHeaderSize);
      memory >> in;
      ASSERT(memory.eof());
    }
  }
  while(size > 0);
}

