/**
 * @file DebugHandler.cpp
 *
 * Class for debug communication over a TCP connection
 *
 * @author Thomas Röfer
 */

#ifdef TARGET_ROBOT
#include "DebugHandler.h"
#include "Platform/BHAssert.h"
#include "Streaming/InStreams.h"
#include "Streaming/OutStreams.h"
#include <limits>

DebugHandler::DebugHandler(MessageQueue& in, MessageQueue& out, int maxPacketSendSize, int maxPacketReceiveSize) :
  TcpConnection(0, 9999, TcpConnection::receiver, maxPacketSendSize, maxPacketReceiveSize),
  in(in),
  out(out)
{}

void DebugHandler::communicate(bool send)
{
  if(send && !sendData && !out.empty())
  {
    sendSize = out.size() + 8;
    OutBinaryMemory memory(sendSize);
    memory << out;
    sendData = reinterpret_cast<unsigned char*>(memory.obtainData());
    out.clear();
  }

  unsigned char* receivedData;
  int receivedSize = 0;

  ASSERT(sendSize <= std::numeric_limits<int>::max());
  if(sendAndReceive(sendData, static_cast<int>(sendSize), receivedData, receivedSize) && sendSize)
  {
    delete [] sendData;
    sendData = nullptr;
    sendSize = 0;
  }

  if(receivedSize > 0)
  {
    InBinaryMemory memory(receivedData);
    memory >> in;
    delete [] receivedData;
  }
}
#endif
