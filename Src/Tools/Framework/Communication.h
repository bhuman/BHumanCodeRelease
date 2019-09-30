/**
 * @file Tools/Framework/Communication.h
 *
 * This file declares classes related to the communication between threads.
 *
 * @author Thomas Röfer
 * @author Jan Fiedler
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Platform/Thread.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/InStreams.h"
#include <cstdlib>

class ThreadFrame;

/**
 * Is only for a dummy sender and sends nothing.
 * It is used for debug senders where only their template parameter is used.
 */
namespace Communication
{
  static const std::string dummy("Dummy");
}

/**
 * @class ReceiverBase
 *
 * The class is a helper that encapsulates all template independent functions from Receiver.
 */
class ReceiverBase
{
public:
  const std::string senderThreadName; /**< The name of the sender thread. */

protected:
  ThreadFrame* thread;   /**< The thread this receiver is associated with. */
  void* packet[3];           /**< A triple buffer for received packets. */
  volatile int reading = 0;   /**< Index of packet reserved for reading. */
  volatile int actual = 0;    /**< Index of packet that is the most actual. */

public:
  /**
   * The constructor.
   * @param thread The thread that should be notified that a packet has arrived.
   * @param senderThreadName The name of the sender thread.
   */
  ReceiverBase(ThreadFrame* thread, const std::string& senderThreadName) :
    senderThreadName(senderThreadName), thread(thread)
  {
    for(int i = 0; i < 3; ++i)
      packet[i] = 0;
  }

  virtual ~ReceiverBase()
  {
    for(int i = 0; i < 3; ++i)
      if(packet[i])
        std::free(packet[i]);
  }

  /**
   * The function sets the packet.
   *
   * @param p The packet.
   */
  void setPacket(void* p);

  /**
   * The function determines whether the receiver has a pending packet.
   *
   * @return Is there still an unprocessed packet?
   */
  bool hasPendingPacket() const { return packet[actual] != 0; }
};

/**
 * @class Receiver
 *
 * The template class implements a receiver, which receive all messages of a thread.
 * A receiver is an object that reads packets from another thread.
 */
template<typename PacketType>
class Receiver : public ReceiverBase, public PacketType
{
public:
  /**
   * The constructor.
   * @param thread The thread that should be notified that a packet has arrived.
   * @param senderThreadName The name of the sender thread.
   */
  Receiver(ThreadFrame* thread, const std::string& senderThreadName) :
    ReceiverBase(thread, senderThreadName) {}

  /**
   * The function checks whether a new packet has arrived.
   */
  void checkForPacket()
  {
    reading = actual;
    if(packet[reading])
    {
      PacketType& data = *static_cast<PacketType*>(this);
      InBinaryMemory memory(packet[reading]);
      memory >> data;
      std::free(packet[reading]);
      packet[reading] = 0;
    }
  }
};

/**
 * @class DebugReceiver
 *
 * The template class implements a receiver for debug packets.
 * The receiver has a size.
 */
template<typename PacketType>
class DebugReceiver : public Receiver<PacketType>
{
public:
  /**
   * The constructor.
   * @param thread The thread that should be notified that a packet has arrived.
   * @param senderThreadName The name of the sender thread.
   * @param size The maximum size of the queue in Bytes.
   */
  DebugReceiver(ThreadFrame* thread, const std::string& senderThreadName, unsigned size = 0) :
    Receiver<PacketType>(thread, senderThreadName)
  {
    if(size > 0)
      PacketType::setSize(size);
  }
};

/**
 * @class Sender
 *
 * The template class implements a sender, which sends all messages of a thread.
 * A sender is an object that sends packets to another thread.
 */
template<typename PacketType>
class Sender : public PacketType
{
public:
  const std::string receiverThreadName; /**< The name of the receiver thread. */

private:
  Receiver<PacketType>& receiver; /**< The recipient of the packets. */

public:
  /**
   * The constructor.
   * @param receiver The receiver that is attached to this sender.
   * @param receiverThreadName The name of the receiver thread.
   */
  Sender(Receiver<PacketType>& receiver, const std::string& receiverThreadName) :
    receiverThreadName(receiverThreadName), receiver(receiver) {}

  virtual ~Sender() = default;

  /**
   * The function sends a packet to the receiver.
   */
  void send()
  {
    // Dummy Sender does not send anything
    if(receiverThreadName == Communication::dummy)
      return;
    const PacketType& data = *static_cast<const PacketType*>(this);
    OutBinaryMemory stream(16384);
    stream << data;
    receiver.setPacket(stream.obtainData());
  }

  /**
   * Returns whether a new packet was requested from the sender.
   * This is always true if this is a blocking sender.
   *
   * @return Has a new packet been requested?
   */
  bool requestedNew() const
  {
    return (!receiver.hasPendingPacket());
  }
};

/**
 * @class DebugSenderBase
 *
 * The base class for the debug packet sender.
 * It only contains a flag that signals to all debug packet senders that the current thread
 * is terminating and they should abort blocking actions.
 */
class DebugSenderBase
{
protected:
  static bool terminating; /**< Is the current thread terminating? */

  friend class RoboCupCtrl; /**< RoboCupCtrl will set this flag. */
};

/**
 * @class DebugSender
 *
 * The template class implements a sender for debug packets.
 * It ensures that only a packet is sent if it is not empty.
 */
template<typename PacketType>
class DebugSender : public Sender<PacketType>, private DebugSenderBase
{
public:
  /**
   * The constructor.
   * @param receiver The receiver that is attached to this sender.
   * @param receiverThreadName The name of the receiver thread.
   * @param size The maximum size of the queue in Bytes.
   * @param reserveForInfrastructure Non-infrastructure messages will be rejected if
   *                                 less than this number of bytes is free.
   */
  DebugSender(DebugReceiver<PacketType>& receiver, const std::string& receiverThreadName,
              unsigned size = 0, unsigned reserveForInfrastructure = 0) :
    Sender<PacketType>(receiver, receiverThreadName)
  {
    if(size > 0)
      PacketType::setSize(size, reserveForInfrastructure);
  }

  /**
   * Marks the packet for sending and transmits it to all receivers that already requested for it.
   * All other receiver may get it later if they request for it before the packet is changed.
   * In function will only send a packet if it is not empty.
   *
   * @param block Whether to block when the packet cannot be send immediatly
   */
  void send(bool block = false)
  {
    if(!Sender<PacketType>::isEmpty())
    {
      bool requestedNew = Sender<PacketType>::requestedNew();
      if(block)
        while(!requestedNew && !terminating)
        {
          Thread::yield();
          requestedNew = Sender<PacketType>::requestedNew();
        }
      if(requestedNew)
      {
        Sender<PacketType>::send();
        Sender<PacketType>::clear();
      }
    }
  }
};
