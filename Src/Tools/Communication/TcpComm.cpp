/**
 * @file Platform/Common/TcpComm.cpp
 *
 * Implementation of class TcpComm.
 *
 * @author Thomas Röfer
 */

#include "TcpComm.h"
#include "Platform/BHAssert.h"

#include <cerrno>
#include <fcntl.h>

#ifdef WINDOWS
#define ERRNO WSAGetLastError()
#define RESET_ERRNO WSASetLastError(0)
#define NON_BLOCK(socket) ioctlsocket(socket, FIONBIO, reinterpret_cast<u_long*>(const_cast<char*>("NONE")))
#define CLOSE(socket) closesocket(socket)
#undef EWOULDBLOCK
#define EWOULDBLOCK WSAEWOULDBLOCK
#undef EINPROGRESS
#define EINPROGRESS WSAEINPROGRESS

struct _WSAFramework
{
  _WSAFramework()
  {
    WORD wVersionRequested = MAKEWORD(2, 2);
    WSADATA wsaData;
    WSAStartup(wVersionRequested, &wsaData);
  }
  ~_WSAFramework() { WSACleanup(); }
} _wsaFramework;

#else

#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define ERRNO errno
#define RESET_ERRNO errno = 0
#define NON_BLOCK(socket) fcntl(socket,F_SETFL,O_NONBLOCK)
#define CLOSE(socket) close(socket)

#endif

#ifndef LINUX
#define MSG_NOSIGNAL 0
#endif

TcpComm::TcpComm(const char* ip, int port, int maxPacketSendSize, int maxPacketReceiveSize) :
  maxPacketSendSize(maxPacketSendSize), maxPacketReceiveSize(maxPacketReceiveSize)
{
  address.sin_family = AF_INET;
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
  address.sin_port = htons(static_cast<unsigned short>(port));
#ifdef __clang__
#pragma clang diagnostic pop
#endif
  if(ip) // connect as client?
    address.sin_addr.s_addr = inet_addr(ip);
  else
  {
    createSocket = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT(createSocket > 0);
    int val = 1;
    setsockopt(createSocket, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&val), sizeof(val));
    address.sin_addr.s_addr = INADDR_ANY;
    VERIFY(bind(createSocket, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr_in)) == 0);
    VERIFY(listen(createSocket, SOMAXCONN) == 0);
    NON_BLOCK(createSocket);
  }
  checkConnection();
}

TcpComm::~TcpComm()
{
  if(connected())
    closeTransferSocket();
  if(createSocket > 0)
    CLOSE(createSocket);
}

bool TcpComm::checkConnection()
{
  if(!connected())
  {
    if(createSocket)
      transferSocket = accept(createSocket, nullptr, nullptr);
    else if(!wasConnected)
    {
      transferSocket = socket(AF_INET, SOCK_STREAM, 0);
      ASSERT(connected());
      if(connect(transferSocket, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr_in)) != 0)
      {
        CLOSE(transferSocket);
        transferSocket = 0;
      }
    }

    if(connected())
    {
      wasConnected = true;
      NON_BLOCK(transferSocket); // switch socket to nonblocking
#ifdef MACOS
      int yes = 1;
      VERIFY(!setsockopt(transferSocket, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes)));
#endif
      if(maxPacketSendSize)
        VERIFY(!setsockopt(transferSocket, SOL_SOCKET, SO_SNDBUF, reinterpret_cast<char*>(&maxPacketSendSize), sizeof(maxPacketSendSize)));
      if(maxPacketReceiveSize)
        VERIFY(!setsockopt(transferSocket, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<char*>(&maxPacketReceiveSize), sizeof(maxPacketReceiveSize)));
      return true;
    }
    else
      return false;
  }
  else
    return true;
}

void TcpComm::closeTransferSocket()
{
  CLOSE(transferSocket);
  transferSocket = 0;
}

bool TcpComm::receive(unsigned char* buffer, int size, bool wait)
{
  if(!checkConnection())
    return false;

  if(!wait)
  {
    RESET_ERRNO;
#ifdef WINDOWS
    char c;
    int received = recv(transferSocket, &c, 1, MSG_PEEK);
    if(!received || (received < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS) ||
       ioctlsocket(transferSocket, FIONREAD, reinterpret_cast<u_long*>(&received)) != 0)
    {
      closeTransferSocket();
      return false;
    }
    else if(received == 0)
      return false;
#else
    int received = static_cast<int>(recv(transferSocket, reinterpret_cast<char*>(buffer), size, MSG_PEEK));
    if(received < size)
    {
      if(!received || (received < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS))
        closeTransferSocket();
      return false;
    }
#endif
  }

  int received = 0;
  while(received < size)
  {
    RESET_ERRNO;

    int received2 = static_cast<int>(recv(transferSocket, reinterpret_cast<char*>(buffer) + received, size - received, 0));

    if(!received2 || (received2 < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS))  // error during reading of packet
    {
      closeTransferSocket();
      return false;
    }
    else if(ERRNO == EWOULDBLOCK || ERRNO == EINPROGRESS) // wait for the rest
    {
      received2 = 0;
      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;
      fd_set rset;
      FD_ZERO(&rset);
      FD_SET(transferSocket, &rset);
      if(select(static_cast<int>(transferSocket + 1), &rset, 0, 0, &timeout) == -1)
      {
        closeTransferSocket();
        return false; // error while waiting
      }
    }
    received += received2;
    overallBytesReceived += received2;
  }
  return true; // ok, data received
}

bool TcpComm::send(const unsigned char* buffer, int size)
{
  if(!checkConnection())
    return false;

  RESET_ERRNO;
  int sent = static_cast<int>(::send(transferSocket, reinterpret_cast<const char*>(buffer), size, MSG_NOSIGNAL));
  if(sent > 0)
  {
    overallBytesSent += sent;
    while(sent < size && (ERRNO == EWOULDBLOCK || ERRNO == EINPROGRESS || ERRNO == 0))
    {
      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;
      fd_set wset;
      FD_ZERO(&wset);
      FD_SET(transferSocket, &wset);
      RESET_ERRNO;
      if(select(static_cast<int>(transferSocket + 1), 0, &wset, 0, &timeout) == -1)
        break;
      RESET_ERRNO;
      int sent2 = static_cast<int>(::send(transferSocket, reinterpret_cast<const char*>(buffer) + sent, size - sent, MSG_NOSIGNAL));
      if(sent2 >= 0)
      {
        sent += sent2;
        overallBytesSent += sent;
      }
    }
  }

  if(ERRNO == 0 && sent == size)
    return true;
  else
  {
    closeTransferSocket();
    return false;
  }
}
