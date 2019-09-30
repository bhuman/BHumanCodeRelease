/**
 * @file Platform/Common/TcpComm.h
 *
 * Declaration of class TcpComm.
 *
 * @author Thomas Röfer
 */

#pragma once

#ifdef WINDOWS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#else
#include <netinet/in.h>
#endif

#ifdef WINDOWS
#define socket_t SOCKET
#else
#define socket_t int
#endif

/**
 * @class TcpComm
 * The class implements a tcp connection.
 */
class TcpComm
{
private:
  socket_t createSocket = 0; /**< The handle of the basic socket. */
  socket_t transferSocket = 0; /**< The handle of the actual transfer socket. */
  sockaddr_in address; /**< The socket address. */
  int overallBytesSent = 0; /**< The overall number of bytes sent so far. */
  int overallBytesReceived = 0; /**< The overall number of bytes received so far. */
  int maxPacketSendSize; /**< The maximum size of an outgoing packet. If 0, this setting is ignored. */
  int maxPacketReceiveSize; /**< The maximum size of an incoming packet. If 0, this setting is ignored. */
  bool wasConnected = false; /**< Whether a transfer connection was established or not */

  /**
   * The method checks whether the connection is available.
   * If not, it tries to reestablish it.
   * @return Can the connection be used now?
   */
  bool checkConnection();

  /**
   * The function closes the transfer socket.
   */
  void closeTransferSocket();

public:
  /**
   * Opens a TCP connection to a remote host.
   * @param ip The ip address of the communication partner. If 0, the port
   *           will be opened as server.
   * @param port The port over which will be communicated.
   * @param maxPacketSendSize The maximum size of an outgoing packet.
   *                           If 0, this setting is ignored.
   * @param maxPacketReceiveSize The maximum size of an incoming packet.
   *                              If 0, this setting is ignored.
   */
  TcpComm(const char* ip, int port, int maxPacketSendSize = 0, int maxPacketReceiveSize = 0);

  ~TcpComm();

  /**
   * The function sends a block of bytes.
   * It will return immediately unless the send buffer is full.
   * @param buffer The bytes to send.
   * @param size The number of bytes to send.
   * @return Was the data successfully sent?
   */
  bool send(const unsigned char* buffer, int size);

  /**
   * The function receives a block of bytes.
   * @param buffer This buffer will be filled with the bytes to receive.
   *               It must provide at least "size" bytes.
   * @param size The number of bytes to receive.
   * @param wait The method will wait until all bytes have been received.
   *             However, it can still fail if the connection is broken.
   * Was the data successfully received?
   */
  bool receive(unsigned char* buffer, int size, bool wait = true);

  /**
   * The function returns the overall number of bytes sent so far by this object.
   * @return The number of bytes sent since this object was created.
   */
  int getOverallBytesSent() const { return overallBytesSent; }

  /**
   * The function returns the overall number of bytes received so far by this object.
   * @return The number of bytes received since this object was created.
   */
  int getOverallBytesReceived() const { return overallBytesReceived; }

  /**
   * The function return whether the connection was successful.
   * @return Was the connection established?
   */
  bool connected() const { return transferSocket > 0; }
};
