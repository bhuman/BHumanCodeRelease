/**
 * @file UdpComm.h
 * Declares a wrapper for a UDP socket.
 * @author Armin Burchardt
 */

#pragma once

struct sockaddr;
struct sockaddr_in;

/**
* @class UdpComm
*/
class UdpComm
{
public:
  /**
  * Constructor.
  */
  UdpComm();

  /**
  * Destructor.
  */
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
  bool setBroadcast(bool enable);

  /**
   * Sets blocking mode.
   */
  bool setBlocking(bool block);

  /**
   * Set Loopback (receive own multicast packets). Default: enabled.
   */
  bool setLoopback(bool);

  /**
  * bind to IN_ADDR_ANY to receive packets
  */
  bool bind(const char* addr, int port);

  /**
  * The function tries to read a package from a socket.
  * @return Number of bytes received or -1 in case of an error.
  */
  int read(char* data, int len);

  /**
   * The function tries to read a package from a socket and report the address of the sender.
   * @return Number of bytes received or -1 in case of an error.
   */
  int read(char* data, int len, sockaddr_in& from);
  
  /**
  * The function writes a package to a socket.
  * @return True if the package was written.
  */
  bool write(const char* data, const int len);

  /**
  * Determines the address that will broadcast to the wifi adapter.
  * @return The wifi broadcast address.
  */
  static const char* getWifiBroadcastAddress();

private:
  struct sockaddr* target;
  int sock;
  bool resolve(const char*, int, struct sockaddr_in*);
};
