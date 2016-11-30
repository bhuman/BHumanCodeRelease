/**
 * @file Platform/Nao/DebugHandler.h
 *
 * Class for debug communication over a TCP connection
 *
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Debugging/TcpConnection.h"
#include "Tools/MessageQueue/MessageQueue.h"

class DebugHandler : TcpConnection
{
private:
  MessageQueue& in; /**< Incoming debug data is stored here. */
  MessageQueue& out; /**< Outgoing debug data is stored here. */

  unsigned char* sendData = nullptr; /**< The data to send next. */
  int sendSize = 0; /**< The size of the data to send next. */

public:
  /**
   * @param in The message queue that stores data received.
   * @param out The message queue containing data to be sent.
   * @param maxPackageSendSize The maximum size of an outgoing package.
   *                           If 0, this setting is ignored.
   * @param maxPackageReceiveSize The maximum size of an incouming package.
   *                              If 0, this setting is ignored.
   */
  DebugHandler(MessageQueue& in, MessageQueue& out, int maxPackageSendSize = 0, int maxPackageReceiveSize = 0);

  /**
   * Delete buffered package if needed.
   */
  ~DebugHandler() {if(sendData) delete [] sendData;}

  /**
   * The method performs the communication.
   * It has to be called at the end of each frame.
   * @param send Send outgoing queue?
   */
  void communicate(bool send);
};
