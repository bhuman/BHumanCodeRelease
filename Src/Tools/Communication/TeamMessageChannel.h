/**
 * @file Tools/Communication/TeamMessageChannel.h
 * The file declares a class for the team communication between robots.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 *
 * based on TeamHandler authored by
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Communication/TeamMessageContainer.h"
#include "Network/UdpComm.h"

/**
 * @class TeamMessageChannel
 * A class for team communication by broadcasting.
 */
class TeamMessageChannel
{
public:
  using Container = TeamMessageContainer;

  /**
   * Constructor.
   * @param in Incoming team message.
   * @param out Outgoing team message.
   */
  TeamMessageChannel(Container& in, Container& out) :
    in(in), out(out) {}

  /**
   * The method starts the actual communication for local communication.
   * @param port The UDP port this handler is listening to.
   * @param localId An identifier for a local robot
   */
  void startLocal(int port, unsigned localId);

  /**
   * The method starts the actual communication on the given port.
   * @param port The UDP port this handler is listening to.
   */
  void start(int port);

  /**
   * The method sends the outgoing message if possible.
   */
  void send();

  /**
   * The method receives packets if available.
   * @return Whether there was a message
   */
  bool receive();

private:
  /** Sets the target of the socket to the WiFi broadcast address (if it exists). */
  void trySetWifiTarget();

  Container& in; /**< Incoming team message is stored here. */
  Container& out; /**< Outgoing team message is stored here. */
  int port = 0; /**< The UDP port this handler is listening to. */
  UdpComm socket; /**< The socket used to communicate. */
  unsigned localId = 0; /**< The id of a local team communication participant or 0 for normal udp communication. */
  bool targetSet = false; /**< Whether the target of the socket has been set. */
};
