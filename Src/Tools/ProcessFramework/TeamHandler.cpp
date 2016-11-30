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
#include "Tools/Debugging/DebugDrawings.h"

#if defined TARGET_ROBOT || defined TARGET_SIM
#include "Modules/Communication/ReceivedSPLStandardMessagesProvider.h"
#else
#include "Representations/Communication/SPLStandardMessage.h"
#endif

TeamHandler::TeamHandler(MessageQueue& in, MessageQueue& out) :
  in(in), out(out)
{}

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

  SPLStandardMessage outMsg(out);

  if(socket.write((char*)&(RoboCup::SPLStandardMessage&) outMsg, offsetof(RoboCup::SPLStandardMessage, data) + outMsg.numOfDataBytes))
    out.clear();

  // Plot usage of data buffer in percent:
  const float usageInPercent = outMsg.numOfDataBytes * 100.f / static_cast<float>(SPL_STANDARD_MESSAGE_DATA_SIZE);
  PLOT("module:TeamHandler:standardMessageDataBufferUsageInPercent", usageInPercent);
}

unsigned TeamHandler::receive()
{
  if(!port)
    return 0; // not started yet

  SPLStandardMessage inMsg;
  int size;
  unsigned remoteIp = 0;
  unsigned receivedSize = 0;

  do
  {
    size = localId ? socket.readLocal((char*)&(RoboCup::SPLStandardMessage&) inMsg, sizeof(RoboCup::SPLStandardMessage))
                   : socket.read((char*)&(RoboCup::SPLStandardMessage&) inMsg, sizeof(RoboCup::SPLStandardMessage), remoteIp);
    if(size >= static_cast<int>(offsetof(RoboCup::SPLStandardMessage, data)) && size <= static_cast<int>(sizeof(RoboCup::SPLStandardMessage)))
    {
      receivedSize = static_cast<unsigned>(size);
      inMsg.toMessageQueue(in, remoteIp, receivedSize - offsetof(RoboCup::SPLStandardMessage, data));

#if defined TARGET_ROBOT || defined TARGET_SIM
      ReceivedSPLStandardMessagesProvider::addMessage(inMsg);
#endif
    }
  }
  while(size > 0);

  return receivedSize;
}
