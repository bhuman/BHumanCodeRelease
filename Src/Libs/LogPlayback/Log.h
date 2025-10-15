/**
 * @file Log.h
 *
 * This file declares a class that represents a log file in memory.
 * It supports iterating through the log file frame by frame and
 * accessing the representations that are stored in them. Message ids
 * are automatically mapped to the current ones. If the type information
 * contained in the log file differs from the current one,
 * representations are converted.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "LogPlayer.h"

class Log
{
  enum State
  {
    unknown, // Not yet decided.
    accept,  // Specification is compatible -> replay.
    convert, // Specification not compatible -> convert.
  };

  const LogPlayer* logPlayer;
  const bool ownsLogPlayer;
  std::array<State, numOfDataMessageIDs> states; /**< How should the corresponding message ids be replayed? */

  /**
   * Private constructor.
   * @param logPlayer The log player.
   * @param ownsLogPlayer Does this object created the log player?
   */
  Log(const LogPlayer& logPlayer, const bool ownsLogPlayer);

public:
  /** A message that can be read. */
  class Message : public MessageQueue::Message
  {
    const Log* log; /**< The log this message belongs to. */
    mutable Streamable* representation = nullptr; /**< A buffer for de-serializing the message. */

    /** De-serialize the current message into an already allocated \c representation. */
    void fillRepresentation() const;

  public:
    /**
     * Constructor.
     * @param buffer The position of the message in the queue's buffer.
     */
    Message(const MessageQueue::Message& message, const Log* log) : MessageQueue::Message(message), log(log) {}

    /**
     * Copy constructor. \c representation is not copied.
     * @param other The object this one is copied from.
     */
    Message(const Message& other) : MessageQueue::Message(other), log(other.log) {}

    /**
     * Move constructor. \c representation is moved to this object.
     * @param other The object that is moved to this one.
     */
    Message(Message&& other) : MessageQueue::Message(other), log(other.log), representation(other.representation)
    {
      other.representation = nullptr;
    }

    /** Destructor. Free's \c representation if necessary. */
    ~Message();

    /**
     * Assignment operator. \c representation is not copied.
     * @param other The object this one is copied from.
     */
    Message& operator=(const Message& other) {return *new(this) Message(other);}

    /**
     * Returns the message id, i.e. a constant representing its type.
     * @return The id, already converted to the message ids currently declared.
     */
    MessageID id() const {return log->logPlayer->id(*this);}

    /**
     * Provide a de-serialized representation of the current message.
     * @tparam T The type of the representation. Must match this message.
     * @return The message streamed into a representation.
     */
    template<typename T> operator const T&() const
    {
      ASSERT(TypeRegistry::getEnumName(id()) + 2 == TypeRegistry::demangle(typeid(T).name()));
      if(!representation)
      {
        representation = new T;
        fillRepresentation();
      }
      return *static_cast<const T*>(representation);
    }

    /**
     * Provide a de-serialized representation of the current message.
     * This version can only be used with \c idFrameBegin and \c idFrameFinished .
     * @return The name of the thread of the current frame.
     */
    operator const std::string&() const;

    /**
     * A shortcut for an explicit type conversion.
     * @tparam T The type of the representation. Must match this message.
     * @return The message streamed into a representation.
     */
    template<typename T> const T& cast() const {return static_cast<const T&>(*this);}
  };

  class Frame
  {
    std::unordered_map<MessageID, Message> messages; /**< The messages found in the frame (except annotations). */
    std::vector<Message> annotationMessages; /**< The annotations found in the frame. */

  public:
    /**
     * Constructor.
     * @param frame The number of the frame.
     * @param log The log this frame belongs to.
     */
    Frame(size_t frame, const Log* log);

    /**
     * Does the frame contain a certain message type?
     * Does not include annotations.
     * @param id The message id representing the message type.
     * @return Is that type present in the frame?
     */
    bool contains(const MessageID id) const;

    /**
     * Return a message of a certain type. Shall only be called
     * if that message type is actually present in the frame.
     * @param id The message id representing the message type.
     * @return The message that allows to access the actual data.
     */
    const Message& operator[](const MessageID id) const;

    /**
     * Returns the annotation messages in the current frame.
     * @return The messages that can be unpacked to annotations.
     */
    const std::vector<Message>& annotations() const {return annotationMessages;}
  };

  /**
   * A constant forward iterator to enumerate the frame of the log. However,
   * the operator \c * returns an object by value, not a reference to an
   * existing object, because there is no such object in the log, only the
   * ability to indirectly access the messages of the frame by streaming
   * them on demand. Therefore, there also is no operator \c -> .
   */
  class const_iterator
  {
    size_t frame;
    const Log* log; /**< The log this iterator belongs to. */
    friend class Log; /**< \c begin() and \c end() call private constructor. */

    /**
     * Private constructor.
     * @param frame The number of the frame in the log.
     * @param log The log this iterator belongs to.
     */
    const_iterator(const size_t frame, const Log* log) : frame(frame), log(log) {}

  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = Frame;
    using difference_type = std::ptrdiff_t;
    using pointer = void;
    using reference = void;

    const_iterator(const const_iterator& other) : frame(other.frame), log(other.log) {}
    const_iterator& operator=(const const_iterator& other) {return *new(this) const_iterator(other);}
    Frame operator*() const {return Frame(frame, log);}
    bool operator==(const const_iterator& other) const {return frame == other.frame && log == other.log;}
    bool operator!=(const const_iterator& other) const {return frame != other.frame || log != other.log;}
    const_iterator& operator++() {++frame; return *this;}
    const_iterator operator++(int) {const_iterator result(*this); ++frame; return result;}
    const_iterator& operator+=(std::ptrdiff_t offset) {frame += offset; return *this;}
    const_iterator& operator-=(std::ptrdiff_t offset) {frame -= offset; return *this;}
    const_iterator operator+(std::ptrdiff_t offset) const {const_iterator result(*this); return result += offset;}
    const_iterator operator-(std::ptrdiff_t offset) const {const_iterator result(*this); return result -= offset;}
    std::ptrdiff_t operator-(const const_iterator& other) const {return frame - other.frame;}
  };

  /**
   * Opens a log file.
   * @param filename The filename of the log to be opened.
   */
  Log(const std::string& filename);

  /**
   * Uses the log file in an existing log player.
   * @param logPlayer The existing log player. The data in that log player
   *                  is not allowed to change while this objects exists.
   */
  Log(const LogPlayer& logPlayer) : Log(logPlayer, false) {}

  /** Deletes the log player if this object created it. */
  ~Log();

  /**
   * Returns an iterator to the first message in the queue.
   * @return A constant iterator.
   */
  const_iterator begin() const {return const_iterator(0, this);}

  /**
   * Returns an iterator that points behind the last message in the queue.
   * @return A constant iterator.
   */
  const_iterator end() const {return const_iterator(logPlayer->frames(), this);}
};
