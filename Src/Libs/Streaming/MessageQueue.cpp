/**
 * @file MessageQueue.cpp
 *
 * This file implements a class that a bag of streamed data (see header for details).
 *
 * @author Thomas Röfer
 */

#include "MessageQueue.h"

void MessageQueue::OutQueue::open(MessageID id, MessageQueue& queue)
{
  maxCapacity = queue.calcMaxCapacity(id);
  if(queue.ensureCapacity(queue.used + sizeof(MessageHeader), maxCapacity))
  {
    this->queue = &queue;
    originalSize = queue.used;
    const MessageHeader header = {{{id, 0}}};
    *reinterpret_cast<MessageHeader*>(queue.buffer + originalSize) = header;
    queue.used += sizeof(MessageHeader);
  }
  else
    this->queue = nullptr;
}

void MessageQueue::OutQueue::writeToStream(const void* p, size_t size)
{
  if(queue)
  {
    if(queue->ensureCapacity(queue->used + size, maxCapacity))
    {
      std::memcpy(queue->buffer + queue->used, p, size);
      queue->used += size;
      reinterpret_cast<MessageHeader*>(queue->buffer + originalSize)->size += static_cast<unsigned>(size);
    }
    else
    {
      queue->used = originalSize;
      queue = nullptr;
    }
  }
}

size_t MessageQueue::calcMaxCapacity(MessageID id) const
{
  constexpr unsigned unprotected = bit(idDebugDrawing - numOfDataMessageIDs)
    | bit(idDebugDrawing3D - numOfDataMessageIDs)
    | bit(idDebugImage - numOfDataMessageIDs)
    | bit(idPlot - numOfDataMessageIDs)
    | bit(idText - numOfDataMessageIDs);

  return id <= idFrameFinished || (id >= numOfDataMessageIDs && !(bit(id - numOfDataMessageIDs) & unprotected))
         ? maxCapacity : maxCapacity - protectedCapacity;
}

bool MessageQueue::ensureCapacity(size_t capacity, size_t maxCapacity)
{
  if(capacity > maxCapacity)
    return false;
  else if(capacity <= this->capacity)
    return true;
#ifndef TARGET_ROBOT
  else
  {
    size_t newCapacity = std::max(static_cast<size_t>(1), this->capacity);
    while(newCapacity < capacity)
      newCapacity *= 2;
    newCapacity = std::min(newCapacity, this->maxCapacity);
    char* newBuffer = static_cast<char*>(realloc(buffer, newCapacity));
    if(newBuffer)
    {
      buffer = newBuffer;
      this->capacity = newCapacity;
      return true;
    }
  }
#endif
  return false;
}

void MessageQueue::copyMessages(size_t size, const std::function<void(void*, size_t)>& copy)
{
  MessageHeader header;
  while(size > 0)
  {
    copy(&header, sizeof(header));
    if(ensureCapacity(used + sizeof(MessageHeader) + header.size, calcMaxCapacity(header.id)))
    {
      *reinterpret_cast<MessageHeader*>(buffer + used) = header;
      used += sizeof(MessageHeader);
      copy(buffer + used, header.size);
      used += header.size;
    }
    else
      copy(nullptr, header.size);
    size -= sizeof(MessageHeader) + header.size;
  }
}

void MessageQueue::read(In& stream)
{
  QueueHeader header;
  stream.read(&header, sizeof(header));
  const size_t size = header.sizeLow | static_cast<size_t>(header.sizeHigh) << 32;
  if(ensureCapacity(used + size, maxCapacity - protectedCapacity))
  {
    stream.read(buffer + used, size);
    used += size;
  }
  else
    copyMessages(size, [&](void* dest, size_t size)
    {
      if(dest)
        stream.read(dest, size);
      else
        stream.skip(size);
    });
}

void MessageQueue::write(Out& stream) const
{
  const QueueHeader header = {used, 0, used >> 32};
  stream.write(&header, sizeof(header));
  append(stream);
}

MessageQueue::MessageQueue()
#ifdef TARGET_ROBOT
: capacity(0),
  maxCapacity(0),
  buffer(nullptr)
#else
  : capacity(16384),
    maxCapacity(0x4000000),
    buffer(static_cast<char*>(malloc(capacity)))
#endif
{
}

MessageQueue::~MessageQueue()
{
  if(ownBuffer)
    free(buffer);
}

MessageQueue& MessageQueue::operator=(const MessageQueue& other)
{
  if(ownBuffer)
    free(buffer);
  ownBuffer = true;
  used = capacity = other.used;
  maxCapacity = other.maxCapacity;
  buffer = static_cast<char*>(malloc(capacity));
  std::memcpy(buffer, other.buffer, used);
  return *this;
}

MessageQueue& MessageQueue::operator<<(const std::pair<const_iterator, const_iterator>& range)
{
  size_t srcSize = range.second - range.first;
  if(ensureCapacity(used + srcSize, maxCapacity - protectedCapacity))
  {
    std::memcpy(buffer + used, range.first.current, srcSize);
    used += srcSize;
  }
  else
  {
    const char* src = range.first.current;
    copyMessages(srcSize, [&](void* dest, size_t size)
    {
      if(dest)
        std::memcpy(dest, src, size);
      src += size;
    });
  }
  return *this;
}

void MessageQueue::clear()
{
  used = 0;
  if(!ownBuffer && !capacity)
  {
    capacity = 16384;
    buffer = static_cast<char*>(malloc(capacity));
    ownBuffer = true;
  }
}

void MessageQueue::resize(size_t size)
{
  ASSERT(used >= size);
  used = size;
}

void MessageQueue::reserve(size_t capacity, size_t protectedCapacity)
{
#ifdef TARGET_ROBOT
  ASSERT(!buffer);
  VERIFY(buffer = static_cast<char*>(malloc(capacity)));
  this->capacity = capacity;
#else
  ASSERT(capacity >= used);
  ASSERT(ownBuffer);
  if(capacity < this->capacity)
  {
    VERIFY(buffer = static_cast<char*>(realloc(buffer, capacity)));
    this->capacity = capacity;
  }
#endif
  maxCapacity = capacity;
  this->protectedCapacity = protectedCapacity;
}

void MessageQueue::setBuffer(const char* buffer, size_t size)
{
  setBuffer(const_cast<char*>(buffer), size, 0);
}

void MessageQueue::setBuffer(char* buffer, size_t size, size_t capacity)
{
  if(ownBuffer)
    free(this->buffer);
  this->buffer = buffer;
  used = size;
  this->capacity = capacity;
  if(capacity)
    maxCapacity = capacity;
  ownBuffer = false;
}

void MessageQueue::append(Out& stream) const
{
  stream.write(buffer, used);
}

void MessageQueue::filter(const std::function<bool(const_iterator)>& keep)
{
  auto begin = this->begin();
  auto end = this->end();
  bool hadOwnBuffer = ownBuffer;
  clear();
  for(auto i = begin; i != end;)
  {
    auto message = i++;
    if(keep(message))
    {
      if(message - begin == used && hadOwnBuffer)
        used += sizeof(MessageHeader) + (*message).size();
      else
        *this << *message;
    }
  }
}
