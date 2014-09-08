/**
* @file Tools/ProcessFramework/TeamHandler.cpp
* The file implements a class for team communication between robots.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "TeamHandler.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/InStreams.h"

#if defined(TARGET_ROBOT) || defined(TARGET_SIM)
#include "Modules/Infrastructure/ReceivedSPLStandardMessagesProvider.h"
#else
#include "Representations/Infrastructure/SPLStandardMessage.h"
#endif

TeamHandler::TeamHandler(MessageQueue& in, MessageQueue& out) :
  in(in),
  out(out),
  port(0), localId(0)
{
}

void TeamHandler::startLocal(int port, unsigned localId)
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

  char buffer[sizeof(RoboCup::SPLStandardMessage)];

  SPLStandardMessage outMsg;
  const unsigned size = outMsg.fromMessageQueue(out);

  OutBinaryMemory memory(buffer);
  memory << outMsg;

  if(socket.write(buffer, size))
    out.clear();
}

unsigned TeamHandler::receive()
{
  if(!port)
    return 0; // not started yet

  char buffer[sizeof(RoboCup::SPLStandardMessage)];
  int size;
  unsigned remoteIp = 0;
  unsigned receivedSize = 0;

  do
  {
    size = localId ? socket.readLocal(buffer, sizeof(buffer)) : socket.read(buffer, sizeof(buffer), remoteIp);
    if(size >= (int)(sizeof(RoboCup::SPLStandardMessage) - SPL_STANDARD_MESSAGE_DATA_SIZE) && size <= (int)(sizeof(RoboCup::SPLStandardMessage)))
    {
      receivedSize = (unsigned) size;

      SPLStandardMessage inMsg;
      InBinaryMemory memory(buffer, size);
      memory >> inMsg;

      inMsg.toMessageQueue(in, remoteIp);

#if defined(TARGET_ROBOT) || defined(TARGET_SIM)
      ReceivedSPLStandardMessagesProvider::addMessage(inMsg);
#endif
    }
  }
  while(size > 0);

  return receivedSize;
}