/**
 * @file Platform/Common/UdpComm.h
 * Wrapper for an udp socket.
 * \author Armin
 */

#pragma once

struct sockaddr;
struct sockaddr_in;

#ifdef WINDOWS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#endif
#include <string>

#ifdef WINDOWS
#define socket_t SOCKET
#else
#define socket_t int
#endif

/**
 * @class UdpComm
 */
class UdpComm
{
private:
  sockaddr* target;
  socket_t sock;

public:
  UdpComm();

  ~UdpComm();

  /**
   * Set default target address.
   * @param ip The ip address of the host system.
   * @param port The port used for the connection.
   * \return Does a connection exist?
   */
  bool setTarget(const char* ip, int port);

  /**
   * Set broadcast mode.
   */
  bool setBroadcast(bool);

  bool setBlocking(bool);

  /**
   * Set multicast mode (please use multicast addresses to avoid confusion).
   */
  bool joinMulticast(const char*);

  /**
   * Set Time-To-Live (router hops).
   */
  bool setTTL(const char);

  /**
   * Set Loopback (receive own multicast packets). Default: enabled.
   */
  bool setLoopback(bool);

  bool setRcvBufSize(unsigned int);

  /**
   * bind to IN_ADDR_ANY to receive packets
   */
  bool bind(const char* addr, int port);

  /**
   * The function tries to read a packet from a socket.
   * @return Number of bytes received or -1 in case of an error.
   */
  int read(char* data, int len, unsigned int& ip);

  /**
   * The function tries to read a packet from a socket.
   * @return Number of bytes received or -1 in case of an error.
   */
  int read(char* data, int len);

  /**
   * The function tries to read a packet from a socket.
   * It only accepts a packet from this host.
   * @return Number of bytes received or -1 in case of an error.
   */
  int readLocal(char* data, int len);

  /**
   * The function writes a packet to a socket.
   * @return True if the packet was written.
   */
  bool write(const char* data, const int len);

  static std::string getWifiBroadcastAddress();

  static unsigned char getLastByteOfIP();

private:
  bool resolve(const char*, int, struct sockaddr_in*);
};
