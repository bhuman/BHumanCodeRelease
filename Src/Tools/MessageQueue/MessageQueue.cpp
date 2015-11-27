/**
 * @file MessageQueue.cpp
 *
 * Implementation of class MessageQueue and helper classes
 *
 * @author Martin Lötzsch
 */

#include <cstring>

#include "MessageQueue.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/Debugging.h"

void MessageQueue::handleAllMessages(MessageHandler& handler)
{
  for(int i = 0; i < queue.numberOfMessages; ++i)
  {
    queue.setSelectedMessageForReading(i);
    in.config.reset();
    in.text.reset();
    handler.handleMessage(in);
  }
}

void MessageQueue::copyAllMessages(MessageQueue& other)
{
  if(queue.usedSize >= MessageQueueBase::headerSize)
  {
    char* dest = other.queue.reserve(queue.usedSize - MessageQueueBase::headerSize);
    if(dest && !other.queue.mappedIDs)
    {
      memcpy(dest - MessageQueueBase::headerSize, queue.buf, queue.usedSize);
      other.queue.numberOfMessages += queue.numberOfMessages;
      other.queue.usedSize += queue.usedSize;
      other.queue.writePosition = 0;
    }
    else // Not all messages fit in there, so try step by step (some will be missing).
      for(int i = 0; i < queue.numberOfMessages; ++i)
        copyMessage(i, other);
  }
}

void MessageQueue::moveAllMessages(MessageQueue& other)
{
  copyAllMessages(other);
  clear();
}

void MessageQueue::patchMessage(int message, int index, char value)
{
  queue.setSelectedMessageForReading(message);
  ASSERT(index >= 0 && index < queue.getMessageSize());
  const_cast<char*>(queue.getData())[index] = value;
}

void MessageQueue::copyMessage(int message, MessageQueue& other)
{
  queue.setSelectedMessageForReading(message);
  other.out.bin.write(queue.getData(), queue.getMessageSize());
  other.out.finishMessage(queue.getMessageID());
}

void MessageQueue::write(Out& stream) const
{
  stream << queue.usedSize << queue.numberOfMessages;
  stream.write(queue.buf, queue.usedSize);
}

void MessageQueue::writeAppendableHeader(Out& stream) const
{
  stream << -1 << -1;
}

void MessageQueue::append(Out& stream) const
{
  stream.write(queue.buf, queue.usedSize);
}

void MessageQueue::append(In& stream)
{
  unsigned usedSize,
           numberOfMessages;
  stream >> usedSize >> numberOfMessages;
  // Trying a direct copy. This is hacked, but fast.
  char* dest = numberOfMessages == static_cast<unsigned>(-1) ? nullptr : queue.reserve(usedSize - MessageQueueBase::headerSize);
  if(dest)
  {
    stream.read(dest - MessageQueueBase::headerSize, usedSize);
    queue.numberOfMessages += numberOfMessages;
    queue.usedSize += usedSize;
    queue.writePosition = 0;
  }
  else // Not all messages fit in there, so try step by step (some will be missing).
    for(unsigned i = 0; numberOfMessages == static_cast<unsigned>(-1) ? !stream.eof() : i < numberOfMessages; ++i)
    {
      unsigned char id = 0;
      unsigned int size = 0;
      stream >> id;

      stream.read(&size, 3);

      if((id >= numOfDataMessageIDs || size == 0) && numberOfMessages == static_cast<unsigned>(-1))
      {
        OUTPUT_WARNING("MessageQueue: Logfile appears to be broken. Skipping rest of file. Read messages: " << queue.numberOfMessages << " read size:" << queue.usedSize);
        break;
      }

      char* dest = numberOfMessages != static_cast<unsigned>(-1) || id < numOfDataMessageIDs ? queue.reserve(size) : nullptr;
      if(dest)
      {
        stream.read(dest, size);
        out.finishMessage(MessageID(id));
      }
      else
        stream.skip(size);
    }
}

Out& operator<<(Out& stream, const MessageQueue& messageQueue)
{
  messageQueue.write(stream);
  return stream;
}

In& operator>>(In& stream, MessageQueue& messageQueue)
{
  messageQueue.append(stream);
  return stream;
}

void operator>>(InMessage& message, MessageQueue& queue)
{
  queue.out.bin.write(message.getData(), message.getMessageSize());
  queue.out.finishMessage(message.getMessageID());
}

char* MessageQueue::getStreamedData()
{
  ((unsigned*)queue.buf)[-2] = queue.usedSize;
  ((unsigned*)queue.buf)[-1] = queue.numberOfMessages;
  return queue.buf - MessageQueueBase::queueHeaderSize;
}
