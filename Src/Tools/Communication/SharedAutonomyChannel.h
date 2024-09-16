/**
 * @file Tools/Communication/SharedAutonomyChannel.h
 * The file declares a class for the communication between the remote
 * PC and the robot in the Shared Autonomy Challenge.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Thomas Röfer
 */

#pragma once

#include "Network/UdpComm.h"

class SharedAutonomyChannel
{
public:
  static constexpr const char* sendBack = ""; /**< Unicast packets to sender. */

  /**
   * The method starts the actual communication for local communication.
   * @param port The UDP port this handler is listening to.
   * @param localId An identifier for a local robot
   */
  void startLocal(int port, unsigned localId);

  /**
   * The method starts the actual communication on the given port.
   * @param port The UDP port this handler is listening to.
   * @param target The target address. Broadcast is supported. If \c sendBack,
   *               unicast will be used, by sending the packets to the ip from
   *               which packets were received.
   */
  void start(int port, const std::string& target);

  /**
   * The method sends the outgoing message if possible.
   * @param data The data to send.
   * @param length The number of bytes to send.
   */
  void send(const char* data, int length);

  /**
   * Receive a packet if available.
   * @param data The buffer to write the received data into.
   * @param length The length of the buffer. Additional data will be dropped.
   * @return The number of bytes actually read, -1 if something went wrong, and -2 if the channel has not been opened yet.
   */
  int receive(char* data, int length);

private:
  int port = 0; /**< The UDP port this handler is listening to. */
  UdpComm socket; /**< The socket used to communicate. */
  unsigned localId = 0; /**< The id of a local communication participant or 0 for normal udp communication. */
  std::string target; /**< The target address or \c sendBack to return packets to sender. */
};
