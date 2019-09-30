/**
 * @file OutMessage.h
 *
 * Declaration of class OutMessageQueue, OutBinaryMessage, OutTextMessage and OutMessage.
 *
 * Include that file in implementation files that only want to write to MessageQueues.
 * (Usually it lasts to include "Tools/Debugging/Debugging.h")
 *
 * @author Martin Lötzsch
 */

#pragma once

#include "Tools/Streams/OutStreams.h"
#include "MessageIDs.h"

class InMessage;
class MessageQueue;
class MessageQueueBase;

/**
 * @class OutMessageQueue
 *
 * A PhysicalOutStream that writes the data to a MessageQueue.
 */
class OutMessageQueue : public PhysicalOutStream
{
private:
  /** The queue where the data is written to */
  MessageQueueBase* queue = nullptr;

protected:
  /**
   * opens the stream.
   * @param q A pointer to the message queue base
   */
  void open(MessageQueueBase* q);

  /**
   * The function writes a number of bytes into a physical stream.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   */
  void writeToStream(const void* p, size_t size) override;
};

/**
 * @class OutBinaryMessage
 *
 * A binary stream into a message queue.
 */
class OutBinaryMessage : public OutStream<OutMessageQueue, OutBinary>
{
public:
  /**
   * Constructor
   * @param q A pointer to the message queue base
   */
  OutBinaryMessage(MessageQueueBase* q);

  /**
   * The function returns whether this is a binary stream.
   * @return Does it output data in binary format?
   */
  bool isBinary() const override {return true;}
};

/**
 * @class OutTextMessage
 *
 * A text stream into a message queue.
 */
class OutTextMessage : public OutStream<OutMessageQueue, OutText>
{
public:
  /**
   * Constructor
   * @param q A pointer to the message queue base
   */
  OutTextMessage(MessageQueueBase* q);
};

/**
 * @class OutTextRawMessage
 *
 * A text stream into a message queue.
 */
class OutTextRawMessage : public OutStream<OutMessageQueue, OutTextRaw>
{
public:
  /**
   * Constructor
   * @param q A pointer to the message queue base
   */
  OutTextRawMessage(MessageQueueBase* q);
};

/**
 * @class OutMessage
 *
 * An Interface for writing messages into a MessageQueue.
 *
 * Use the bin or text member for formatted writing into a message queue.
 */
class OutMessage
{
private:
  /**
   * The message queue where the messages are written into.
   */
  MessageQueueBase& queue;

public:
  /** An interface for writing binary messages into the queue */
  OutBinaryMessage bin;

  /** An interface for writing text messages into the queue */
  OutTextMessage text;

  /**
   * An interface for writing text messages in a raw style (see class OutTextRaw)
   * into the queue
   */
  OutTextMessage textRaw;

  /**
   * Constructor
   * @param queue A reference to a MessageQueueBase
   */
  OutMessage(MessageQueueBase& queue);

  /**
   * Finishes the message and allows to write a new message.
   * Call that function after the writing of every message.
   * @param id The type id of the message
   * @return true if the message fit into the underlying queue, false otherwise.
   */
  bool finishMessage(MessageID id);

  /**
   * Cancels the current message.
   */
  void cancelMessage();

  /** gives the MessageQueue class access to protected members */
  friend class MessageQueue;

  /** gives the LogPlayer class access to protected members */
  friend class LogPlayer;

  /** gives the InMessage class access to protected members */
  friend class InMessage;

  /** gives the operator that copies a InMessage to another queue access to protected members */
  friend void operator>>(InMessage& message, MessageQueue& queue);

  /** gives the In streaming operator access to protected members */
  friend In& operator>>(In& stream, MessageQueue& messageQueue);
};
