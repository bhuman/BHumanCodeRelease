/**
 * @file MessageQueue.h
 *
 * This file declares a class that represents a bag of streamed data. In the
 * context of this class, each entry is called a message. A message has a type
 * that is represented as a value of the enumeration type \c MessageID. It also
 * has a size. Messages are stored sequentially in a single block of memory.
 * They basically form a single linked list with their size information
 * functioning as the links. Given this structure, the class provides two
 * abstractions in its interface.
 * One the one hand, it stores a block of memory, i.e. it works on bytes.
 * \c reserve() allows to reserve a maximum size, \c size() returns the
 * currently used size, and \c resize() allows the shrink the used size (all in
 * bytes). Also, the iterator arithmetics \c += , \c -= , \c + , and \c - all
 * work on bytes.
 * On the other hand, the iterator operator \c ++ allows to walk through the
 * memory block message-wise and its operator \c * returns an object that
 * represents a single message that can be opened as a stream (using either
 * \c bin() or \c text() ). The three functions \c bin() , \c text() , and
 * \c textRaw() of the message queue itself allow to open a writable stream
 * to append a single message. A message must be completely written before a
 * stream for the next message is requested.
 * If the message queue is already full, messages that should be appended are
 * dropped. Since some messages are more relevant than others, the message
 * queue also supports a protected capacity that is reserved for the more
 * important messages. So some message types are dropped earlier than others.
 * Usually, the memory block containing the messages is maintained by the
 * message queue. However, it is also possible to set an external memory block
 * to support the use of a memory-mapped file as (read-only) storage.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "InStreams.h"
#include "MessageIDs.h"
#include "OutStreams.h"
#include "Streamable.h"
#include <functional>
#include <utility>

class MessageQueue : public Streamable
{
  /**
   * The physical stream for writing to the message queue. It will revert all write
   * operations if the data does not fit into the queue. In that case, \c failed()
   * will return \c true .
   */
  class OutQueue : public PhysicalOutStream
  {
    MessageQueue* queue = nullptr; /**< The message queue this stream is attached to. Is \c nullptr if writing is forbidden (e.g. because the queue is full). */
    size_t originalSize; /**< The size of the queue before the first write operation. Is used to be able to revert write operations in case the queue is full. */
    size_t maxCapacity; /**< The maximum capacity of the queue for the current message. It depends on the type of message that is written. */

  protected:
    /**
     * Opens this stream for writing.
     * @param id The type of this message.
     * @param queue The message queue that is written to.
     */
    void open(MessageID id, MessageQueue& queue);

    /**
     * Writes data to the queue.
     * @param p The address of the data to be written.
     * @param size The number of bytes to be written.
     */
    void writeToStream(const void* p, size_t size) override;

  public:
    /**
     * Did writing to the queue fail because there was not enough space?
     * @return Did it fail?
     */
    bool failed() const {return queue == nullptr;}
  };

  size_t used = 0; /**< The used capacity of the queue in bytes. */
  size_t capacity; /**< The currently allocated capacity in bytes. */
  size_t maxCapacity; /**< The maximum capacity of the queue in bytes. \c capacity cannot grow more than this. */
  size_t protectedCapacity = 0; /**< A part of the maximum capacity that is reserved for certain message types (in bytes). */
  char* buffer = nullptr; /**< The memory block of size \c capacity containing the messages. */
  bool ownBuffer = true; /**< Is the memory block maintained by this class? */

  /**
   * Determines the maximum capacity applicable for a specific message type.
   * @param id The message type.
   * @return The maximum capacity, including or excluding the protected capacity.
   */
  size_t calcMaxCapacity(MessageID id) const;

  /**
   * Ensures that a certain capacity is actually allocated.
   * @param capacity The capacity that is required.
   * @param maxCapacity The maximum capacity that is available for the current message type.
   * @return Is the capacity actually available?
   */
  bool ensureCapacity(size_t capacity, size_t maxCapacity);

  /**
   * Copy messages from a source to the message queue.
   * @param size The number of bytes the messages consist of.
   * @param copy A function that actually copies a single message. The first parameter
   *             is the target address the message should be copied to. If it is
   *             \c nullptr , the message should be skipped. The second parameter
   *             is the size of the message to be copied or skipped.
   */
  void copyMessages(size_t size, const std::function<void(void*, size_t)>& copy);

protected:
  /**
   * Reads a message queue from a stream and appends its messages to this one.
   * Some messages might be dropped if they do not fit.
   * @param stream The stream to read from.
   */
  void read(In& stream) override;

  /**
   * Writes this message queue to a stream.
   * @param stream The stream to write to.
   */
  void write(Out& stream) const override;

public:
  /** The header of a streamed queue. */
  struct QueueHeader
  {
    size_t sizeLow : 32; /**< The lower 32 bits of the size of the queue. */
    size_t messages : 28; /**< Unused. In previous implementations, this contained the number of messages in the queue. */
    size_t sizeHigh : 4; /**< The bits 32..35 of the size of the queue. */
  };

  /**
   * The header of a single message in the queue.
   * Microsoft's compilers only supports packed bit fields if all members have
   * the same type. Therefore this rather strange construct is used, in which
   * \c _ and \c id share the same address.
   */
  struct MessageHeader
  {
    union
    {
      struct
      {
        unsigned _ : 8; /**< A placeholder for the message type. */
        unsigned size : 24; /**< The size of the message in bytes (not including this header). */
      };
      MessageID id;  /**< The message type. */
    };
  };

  /** A message that can be read. */
  class Message
  {
    const char* buffer; /**< The position of the message in the queue's buffer. */
    friend class MessageQueue; /**< \c operator<< need access to \c buffer . */

  public:
    /**
     * Constructor.
     * @param The position of the message in the queue's buffer.
     */
    Message(const char* buffer) : buffer(buffer) {}

    /**
     * Returns the message id, i.e. a constant representing its type.
     * @return The id.
     */
    MessageID id() const {return reinterpret_cast<const MessageHeader*>(buffer)->id;}

    /**
     * Returns the message's size in bytes.
     * @return The size (without the \c MessageHeader ).
     */
    size_t size() const {return reinterpret_cast<const MessageHeader*>(buffer)->size;}

    /**
     * Returns a stream that allows reading the message in binary format.
     * @return The binary stream.
     */
    InBinaryMemory bin() const {return InBinaryMemory(buffer + sizeof(MessageHeader), size());}

    /**
     * Returns a stream that allows reading the message in textual format.
     * @return The textual stream.
     */
    InTextMemory text() const {return InTextMemory(buffer + sizeof(MessageHeader), size());};
  };

  /** Stream for adding a message in binary format. */
  struct OutBinary : public OutStream<OutQueue, ::OutBinary>
  {
    OutBinary(MessageID id, MessageQueue& queue) {open(id, queue);}
  };

  /** Stream for adding a message in textual format. */
  struct OutText : public OutStream<OutQueue, ::OutText>
  {
    OutText(MessageID id, MessageQueue& queue) {open(id, queue);}
  };

  /** Stream for adding a message in raw textual format. */
  struct OutTextRaw : public OutStream<OutQueue, ::OutTextRaw>
  {
    OutTextRaw(MessageID id, MessageQueue& queue) {open(id, queue);}
  };

  /**
   * A constant forward iterator to enumerate the messages of the queue. It
   * overloads the usual operators with some specialties as described at the
   * beginning of this file. However, the operator \c * returns an object by
   * value, not a reference to an existing object, because there is no direct
   * access to the messages in the queue, only the ability to get a stream to
   * read from. Therefore, there also is no operator \c -> .
   */
  class const_iterator
  {
  private:
    const char* current; /**< The position of the message in the queue's buffer. */
    friend class MessageQueue; /**< \c operator<< need access to \c current . */
    friend class LogPlayer; /**< \c playBack need access to \c current . */

  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = Message;
    using difference_type = size_t;
    using pointer = void;
    using reference = void;

    const_iterator(const char* current) : current(current) {}
    const_iterator(const const_iterator& other) : const_iterator(other.current) {}
    const_iterator& operator=(const const_iterator& other) {return *new(this) const_iterator(other.current);}
    Message operator*() const {return Message(current);}
    bool operator==(const const_iterator& other) const {return current == other.current;}
    bool operator!=(const const_iterator& other) const {return current != other.current;}
    const_iterator& operator++() {current += sizeof(MessageHeader) + reinterpret_cast<const MessageHeader*>(current)->size; return *this;}
    const_iterator operator++(int) {const_iterator result(*this); current += sizeof(MessageHeader) + reinterpret_cast<const MessageHeader*>(current)->size; return result;}
    const_iterator& operator+=(std::ptrdiff_t offset) {current += offset; return *this;}
    const_iterator& operator-=(std::ptrdiff_t offset) {current -= offset; return *this;}
    const_iterator operator+(std::ptrdiff_t offset) const {const_iterator result(*this); return result += offset;}
    const_iterator operator-(std::ptrdiff_t offset) const {const_iterator result(*this); return result -= offset;}
    size_t operator-(const const_iterator& other) const {return current - other.current;}
  };

  /**
   * Default constructor.
   * On the desktop, this will already reserve a small capacity for the queue
   * (16 kb).
   * The maximum capacity is set to 64 mb. On the robot, \c reserve must be
   * called before the queue can be used.
   */
  MessageQueue();

  /** The destructor deallocates the queue. */
  ~MessageQueue();

  /**
   * Copies another message queue to this one. Messages previously stored in
   * this message queue are lost. After this operation, the capacity equals the
   * size and the maximum capacity is the same as in the other queue.
   * @param other The queue the messages are copied from.
   * @return This queue.
   */
  MessageQueue& operator=(const MessageQueue& other);

  /**
   * Appends a range of messages to this queue. Messages might be dropped if
   * they do not fit.
   * @param range Iterators marking the first message (inclusive) and the last
   *              message (exclusive) that are appended.
   * @return This queue.
   */
  MessageQueue& operator<<(const std::pair<const_iterator, const_iterator>& range);

  /**
   * Appends the messages of another queue to this one. Messages might be
   * dropped if they do not fit.
   * @param other The other message queue.
   * @return This queue.
   */
  MessageQueue& operator<<(const MessageQueue& other)
    {return *this << std::pair<const_iterator, const_iterator>(other.begin(), other.end());}

  /**
   * Appends a single message to this queue. It might be dropped if it does not
   * fit.
   * @param message The message.
   * @return This queue.
   */
  MessageQueue& operator<<(const Message& message)
    {return *this << std::pair<const_iterator, const_iterator>(message.buffer, message.buffer + sizeof(MessageHeader) + message.size());}

  /** Empties the queue. */
  void clear();

  /**
   * Returns the used size of the queue.
   * @return The size in bytes.
   */
  size_t size() const {return used;}

  /**
   * Changes the size of the queue.
   * @param size The new size of the queue. Must be smaller or equal to the
   *             current size and must also be at the end of a message.
   */
  void resize(size_t size);

  /**
   * Is the queue empty?
   * @return Is it?
   */
  bool empty() {return used == 0;}

  /**
   * Defines the maximum capacity of the queue. On the robot, this capacity is
   * immediately allocated and this method must be called exactly once after
   * the construction of this message queue.
   * @param capacity The maximum capacity of the queue in bytes.
   * @param protectedCapacity A part of the maximum capacity that is reserved
   *                          for certain message types (in bytes).
   */
  void reserve(size_t capacity, size_t protectedCapacity = 0);

  /**
   * Sets an external read-only message buffer. Will free the previous buffer
   * if it was managed by the queue.
   * @param buffer The external buffer. The queue does not take ownership of
   *               that buffer, i.e. it will not free it.
   * @param size The size of the external buffer.
   */
  void setBuffer(const char* buffer, size_t size);

  /**
   * Sets an external writable message buffer. Will free the previous buffer
   * if it was managed by the queue.
   * @param buffer The external buffer. The queue does not take ownership of
   *               that buffer, i.e. it will not free it.
   * @param size The size already used in the external buffer.
   * @param capacity The capacity of the external buffer.
   */
  void setBuffer(char* buffer, size_t size, size_t capacity);

  /**
   * Appends the messages of the queue to a stream. In contrast to the
   * operator \c << , no \c QueueHeader will be written.
   * @param stream The stream that is appended.
   */
  void append(Out& stream) const;

  /**
   * Filters messages based on a used-defined criterion.
   * @param keep A function that returns whether a message should be kept. The
   *             parameter passed is the iterator to the message in question.
   */
  void filter(const std::function<bool(const_iterator)>& keep);

  /**
   * Returns an iterator to the first message in the queue.
   * @return A constant iterator.
   */
  const_iterator begin() const {return const_iterator(buffer);}

  /**
   * Returns an iterator that points behind the last message in the queue.
   * @return A constant iterator.
   */
  const_iterator end() const {return const_iterator(buffer + used);}

  /**
   * Returns a binary stream that allows to append a new message.
   * @param id The type of the new message.
   * @return A binary stream.
   */
  OutBinary bin(MessageID id) {return OutBinary(id, *this);}

  /**
   * Returns a textual stream that allows to append a new message.
   * @param id The type of the new message.
   * @return A textual stream.
   */
  OutText text(MessageID id) {return OutText(id, *this);}

  /**
   * Returns a raw textual stream that allows to append a new message.
   * @param id The type of the new message.
   * @return A raw textual stream, i.e. a stream that does not add spaces
   *         between different entries.
   */
  OutTextRaw textRaw(MessageID id) {return OutTextRaw(id, *this);}
};
