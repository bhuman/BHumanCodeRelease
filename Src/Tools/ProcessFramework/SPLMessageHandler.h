/**
 * @file Tools/ProcessFramework/SPLMessageHandler.h
 * The file declares a class for the team communication between robots.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 *
 * based on TeamHandler authored by
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Communication/UdpComm.h"
#include "Tools/Communication/SPLStandardMessageBuffer.h"

namespace RoboCup
{
#include <SPLStandardMessage.h>
}

#define MAX_NUMBER_OF_PARALLEL_RECEIVABLE_SPLSTDMSG 9

/**
 * @class SPLMessageHandler
 * A class for team communication by broadcasting.
 */
class SPLMessageHandler
{
private:
  SPLStandardMessageBuffer<MAX_NUMBER_OF_PARALLEL_RECEIVABLE_SPLSTDMSG>& in; /**< Incoming spl messages are stored here. */
  RoboCup::SPLStandardMessage& out; /**< Outgoing spl message is stored here. */
  int port = 0; /**< The UDP port this handler is listening to. */
  UdpComm socket; /**< The socket used to communicate. */
  unsigned localId = 0; /**< The id of a local team communication participant or 0 for normal udp communication. */

public:
  /**
   * Constructor.
   * @param in Incoming spl standard messages.
   * @param out Outgoing spl standard message.
   */
  SPLMessageHandler(SPLStandardMessageBuffer<MAX_NUMBER_OF_PARALLEL_RECEIVABLE_SPLSTDMSG>& in, RoboCup::SPLStandardMessage& out) :
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
   * @param subnet The subnet the handler is broadcasting to.
   */
  void start(int port, const char* subnet);

  /**
   * The method sends the outgoing message if possible.
   */
  void send();

  /**
   * The method receives packages if available.
   * @return The number of bytes received.
   */
  unsigned receive();
};
