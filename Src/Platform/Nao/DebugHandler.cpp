/**
 * @file Platform/Nao/DebugHandler.cpp
 *
 * Class for debug communication over a TCP connection
 *
 * @author Thomas Röfer
 */

#include "DebugHandler.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/InStreams.h"
#include <limits>

DebugHandler::DebugHandler(MessageQueue& in, MessageQueue& out, int maxPacketSendSize, int maxPacketReceiveSize) :
  TcpConnection(0, 9999, TcpConnection::receiver, maxPacketSendSize, maxPacketReceiveSize),
  in(in),
  out(out)
{}

void DebugHandler::communicate(bool send)
{
  if(send && !sendData && !out.isEmpty())
  {
    sendSize = out.getStreamedSize();
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
