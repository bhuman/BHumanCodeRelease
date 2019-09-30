/**
 * @file MessageQueueBase.h
 * Declaration of the class that performs the memory management for the class MessageQueue.
 * @author Martin Lötzsch
 * @author Thomas Röfer
 */

#pragma once

#include <cstddef>
#include <string>

#include "MessageIDs.h"

class LogPlayer;
class In;
class Out;

/**
 * @class MessageQueueBase
 * The class performs the memory management for the class MessageQueue.
 * On Windows, the queue will grow when needed, on the robot, it will remain constant at
 * the size defined by setSize() and reject further messages.
 */
class MessageQueueBase
{
private:
  static constexpr int headerSize = 4; /**< The size of the header of each message in bytes. */
  static constexpr int queueHeaderSize = 2 * sizeof(unsigned); /**< The size of the header in a streamed queue. */
  char* buf = nullptr; /**< The buffer on that the queue works. */
  size_t* messageIndex = 0; /**< An index of the beginnings of all messages. */
  unsigned char numOfMappedIDs = 0; /**< The number of ids in the translation table. If 0, there is no table. */
  MessageID* mappedIDs = nullptr; /**< The mapping of internal ids to external ids. */
  std::string* mappedIDNames = nullptr; /**< The names of the internal ids. */
  size_t selectedMessageForReadingPosition = 0; /**< The position of the message that is selected for reading. */
  size_t reserveForInfrastructure = 0; /**< Non-infrastructure messages will be rejected if less than this number of bytes is free. */
  size_t maximumSize = 0; /**< The maximum queue size (in bytes). */
  size_t reservedSize = 0; /**< The queue size reserved (in bytes). */
  size_t usedSize = 0; /** The queue size used (in bytes). It is also the position where the next message starts. */
  unsigned writePosition = 0; /**< The current size of the next message. */
  bool writingOfLastMessageFailed = false; /**< If true, then the writing of the last message failed because there was not enough space. */
  int readPosition = 0; /**< The position up to where a message is already read. */
  int lastMessage = 0; /**< Cache the current message in the message queue. */
  int numberOfMessages = 0; /**< The number of messages stored. */

  friend class MessageQueue;
  friend class LogPlayer;

public:
  MessageQueueBase();
  ~MessageQueueBase();

  /**
   * The method sets the size of memory which is allocated for the queue.
   * In the simulator, this is only the maximum size (dynamic allocation).
   * @param size The maximum size of the queue in Bytes.
   * @param reserveForInfrastructure Non-infrastructure messages will be rejected if
   *                                 less than this number of bytes is free.
   */
  void setSize(size_t size, size_t reserveForInfrastructure);

  /**
   * Returns the (maximum) size of the queue.
   */
  size_t getSize() const {return maximumSize;}

  /**
   * The method removes all messages from the queue.
   */
  void clear();

  /**
   * The method removes a message from the queue.
   * @param message The number of the message.
   */
  void removeMessage(int message);

  /**
   * The method adds a number of bytes to the last message in the queue.
   * @param p The address the data is located at.
   * @param size The number of bytes to be written.
   */
  void write(const void* p, size_t size);

  /**
   * The method finishes the last message in the queue.
   * The number of messages is increased, and a new message can be started.
   * @param id The type id of the message.
   * @return true if the message fit into the queue, false otherwise.
   */
  bool finishMessage(MessageID id);

  /**
   * The method cancels the current message.
   */
  void cancelMessage() {writePosition = 0;}

  /**
   * The method returns whether the currently selected message for reading was read completely.
   * @return Has the end of the message been reached?
   */
  bool eof() const {return readPosition == getMessageSize();}

  /**
   * The method reads a number of bytes from the currently selected message for reading.
   * @param p The address the data is written to. Note that p must point to a memory area
   *          that is at least "size" bytes large.
   * @param size The number of bytes to be read.
   */
  void read(void* p, size_t size);

  /**
   * The method gives direct read access to the selected message for reading.
   * @return The address of the first byte of the message
   */
  const char* getData() const {return buf + selectedMessageForReadingPosition + headerSize;}

  /**
   * The method returns the message id of the currently selected message for reading.
   * @return The message id.
   */
  MessageID getMessageID() const;

  /**
   * The method returns the message size of the currently selected message for reading.
   * @return The size in bytes.
   */
  int getMessageSize() const {return (*reinterpret_cast<int*>(buf + selectedMessageForReadingPosition + 1)) & 0xffffff;}

  /**
   * The method returns the number of bytes not read yet in the current message.
   * @return The number of bytes left.
   */
  int getBytesLeftInMessage() const {return getMessageSize() - readPosition;}

  /**
   * The method resets read position of the currently selected message for reading
   * so that the message can be read again.
   */
  void resetReadPosition() {readPosition = 0;}

  /**
   * The method selects a message for reading.
   * @param message The number of the message that is selected.
   */
  void setSelectedMessageForReading(int message);

  /**
   * The method returns the selected message for reading.
   * @return The number of the message that is selected.
   */
  int getSelectedMessageForReading() const {return lastMessage;}

  /**
   * The method deletes older messages from the queue if newer messages of same type
   * are already in the queue. However, some message types remain untouched.
   * This method should not be called during message handling.
   */
  void removeRepetitions();

  /**
   * Write message ids to a stream as text.
   * @param stream The stream to write to.
   * @param numOfMessageIDs The number of message ids to write. They always start with the first one.
   */
  void writeMessageIDs(Out& stream, MessageID numOfMessageIDs = ::numOfDataMessageIDs) const;

  /**
   * Create a message id mapping from a stream. The stream contains the message ids as text.
   * The mapping associates these string with the ids that currently use that name.
   * @param stream The stream to read from.
   */
  void readMessageIDMapping(In& stream);

private:
  /**
   * The method reserves a number of bytes in the message queue.
   * @param size The number of bytes to reserve.
   * @return The address of the reserved space or 0 if there was no room.
   */
  char* reserve(size_t size);

  /**
   * Creates an index of the beginnings of all messages.
   * Note that the index is not automatically updated if
   * the contents change.
   */
  void createIndex();

  /**
   * Frees the index if it exists.
   */
  void freeIndex();
};
