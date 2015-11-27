/**
 * @file Tools/Debugging/TcpConnection.cpp
 *
 * Implementation of class TcpConnection.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "TcpConnection.h"
#include "Platform/BHAssert.h"

void TcpConnection::connect(const char* ip, int port, Handshake handshake, int maxPackageSendSize, int maxPackageReceiveSize)
{
  this->handshake = handshake;
  ack = false;
  client = false;

  tcpComm = new TcpComm(ip, port, maxPackageSendSize, maxPackageReceiveSize);
  ASSERT(tcpComm);
  if(ip)
  {
    if(tcpComm->connected())
      client = true;
    else
    {
      delete tcpComm;
      tcpComm = new TcpComm(0, port, maxPackageSendSize, maxPackageReceiveSize);
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
    // we have received a package, but we don't want to send one now.
    // so, send heartbeat instead to acknowledge that we are still alive
    sendHeartbeat();
  }

  // Try to send data
  if((handshake != receiver || ack) &&
     isConnected() && sendSize > 0)
  {
    if(tcpComm->send((unsigned char*)&sendSize, sizeof(sendSize)) && // sends size of block
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
  return tcpComm->send((unsigned char*)&empty, sizeof(empty));
}

int TcpConnection::receive(unsigned char*& buffer)
{
  int size;
  if(tcpComm->receive((unsigned char*)&size, sizeof(size), false))
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

      buffer = new unsigned char[size]; // get buffer for complete package
      ASSERT(buffer);
      if(!tcpComm->receive(buffer, size, true)) // read complete package (wait and read size bytes)
      {
        delete[] buffer;
        return -1; // error
      }
      else
      {
        ack = true;
        return size; // package received
      }
    }
  }
  else
    return 0; // nothing read, but ok
}
