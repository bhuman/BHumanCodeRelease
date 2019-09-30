/**
 * @file Tools/Debugging/TcpConnection.cpp
 *
 * Implementation of class TcpConnection.
 *
 * @author Thomas Röfer
 */

#include "TcpConnection.h"
#include "Platform/BHAssert.h"

void TcpConnection::connect(const char* ip, int port, Handshake handshake, int maxPacketSendSize, int maxPacketReceiveSize)
{
  this->handshake = handshake;
  ack = false;
  client = false;

  tcpComm = std::make_unique<TcpComm>(ip, port, maxPacketSendSize, maxPacketReceiveSize);
  ASSERT(tcpComm);
  if(ip)
  {
    if(tcpComm->connected())
      client = true;
    else
    {
      tcpComm = std::make_unique<TcpComm>(nullptr, port, maxPacketSendSize, maxPacketReceiveSize);
      ASSERT(tcpComm);
    }
  }
}

bool TcpConnection::sendAndReceive(const unsigned char* dataToSend, int sendSize,
                                   unsigned char*& dataRead, int& readSize)
{
  ASSERT(tcpComm);
  bool connectedBefore = isConnected();
  readSize = receive(dataRead);

  if(handshake == sender &&
     ((readSize > 0 && !sendSize) || (!connectedBefore && isConnected())))
  {
    // we have received a packet, but we don't want to send one now.
    // so, send heartbeat instead to acknowledge that we are still alive
    sendHeartbeat();
  }

  // Try to send data
  if((handshake != receiver || ack) &&
     isConnected() && sendSize > 0)
  {
    if(tcpComm->send(reinterpret_cast<unsigned char*>(&sendSize), sizeof(sendSize)) && // sends size of block
       tcpComm->send(dataToSend, sendSize))                           // sends data
    {
      ack = false;
      return true;
    }
    if(connectedBefore && !isConnected())
      return true; // We cannot reconnect, so we fake success to prevent this packet from being sent again
  }
  return false;
}

bool TcpConnection::sendHeartbeat()
{
  ASSERT(tcpComm);
  int empty = 0;
  return tcpComm->send(reinterpret_cast<unsigned char*>(&empty), sizeof(empty));
}

int TcpConnection::receive(unsigned char*& buffer)
{
  int size;
  if(tcpComm->receive(reinterpret_cast<unsigned char*>(&size), sizeof(size), false))
  {
    if(size == 0)
    {
      ack = true;
      return 0; // nothing to read (maybe heartbeat)
    }
    else
    {
      // prevent from allocating to much buffer
      if(size > MAX_PACKAGE_SIZE)
        return -1;

      buffer = new unsigned char[size]; // get buffer for complete packet
      ASSERT(buffer);
      if(!tcpComm->receive(buffer, size, true)) // read complete packet (wait and read size bytes)
      {
        delete[] buffer;
        return -1; // error
      }
      else
      {
        ack = true;
        return size; // packet received
      }
    }
  }
  else
    return 0; // nothing read, but ok
}
