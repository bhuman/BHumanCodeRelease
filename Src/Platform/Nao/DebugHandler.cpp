/**
 * @file Platform/Nao/DebugHandler.cpp
 *
 * Class for debug communication over a TCP connection
 *
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#include "DebugHandler.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/InStreams.h"

DebugHandler::DebugHandler(MessageQueue& in, MessageQueue& out, int maxPackageSendSize, int maxPackageReceiveSize) :
  TcpConnection(0, 0xA1BD, TcpConnection::receiver, maxPackageSendSize, maxPackageReceiveSize),
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

  if(sendAndReceive(sendData, sendSize, receivedData, receivedSize) && sendSize)
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
