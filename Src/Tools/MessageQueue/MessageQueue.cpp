/**
 * @file MessageQueue.cpp
 *
 * Implementation of class MessageQueue and helper classes
 *
 * @author Martin Lötzsch
 */

#include "MessageQueue.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/Debugging.h"
#include <cstring>

void MessageQueue::handleAllMessages(MessageHandler& handler)
{
  for(int i = 0; i < queue.numberOfMessages; ++i)
  {
    queue.setSelectedMessageForReading(i);
    in.text.reset();
    handler.handleMessage(in);
  }
}

void MessageQueue::copyAllMessages(MessageQueue& other)
{
  if(queue.usedSize >= MessageQueueBase::headerSize)
  {
    if(!queue.mappedIDs)
    {
      char* dest = other.queue.reserve(queue.usedSize - MessageQueueBase::headerSize);
      if(dest)
      {
        memcpy(dest - MessageQueueBase::headerSize, queue.buf, queue.usedSize);
        other.queue.numberOfMessages += queue.numberOfMessages;
        other.queue.usedSize += queue.usedSize;
        other.queue.writePosition = 0;
        return;
      }
    }

    // Copy step by step by step, because not all messages will fit
    // or the message ids should be translated.
    for(int i = 0; i < queue.numberOfMessages; ++i)
      copyMessage(i, other);
  }
}

void MessageQueue::moveAllMessages(MessageQueue& other)
{
  copyAllMessages(other);
  clear();
}

void MessageQueue::patchMessage(int message, int index, const std::string& value)
{
  queue.setSelectedMessageForReading(message);
  ASSERT(index >= 0 && index < queue.getMessageSize());
  // streamed strings have 4 Bytes before the real string
  ASSERT(value.size() + 4 == static_cast<size_t>(queue.getMessageSize()));
  OutBinaryMemory stream(value.size() + 4, const_cast<char*>(queue.getData() + index));
  stream << value;
}

void MessageQueue::copyMessage(int message, MessageQueue& other)
{
  queue.setSelectedMessageForReading(message);
  other.out.bin.write(queue.getData(), queue.getMessageSize());
  other.out.finishMessage(queue.getMessageID());
}

void MessageQueue::write(Out& stream) const
{
  stream << static_cast<unsigned>(queue.usedSize) << ((queue.numberOfMessages & 0x0fffffff) | (static_cast<unsigned>(queue.usedSize >> 4) & 0xf0000000));
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
  size_t usedSize = 0;
  unsigned numberOfMessages;
  stream >> *reinterpret_cast<unsigned*>(&usedSize) >> numberOfMessages;
  usedSize |= static_cast<size_t>(numberOfMessages & 0xf0000000) << 4;
  numberOfMessages &= 0x0fffffff;
  if(numberOfMessages == 0x0fffffff)
    numberOfMessages = static_cast<unsigned>(-1);
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

      if(id >= numOfDataMessageIDs && numberOfMessages == static_cast<unsigned>(-1))
      {
        OUTPUT_WARNING("MessageQueue: Log file appears to be broken. Skipping rest of file. Read messages: " << queue.numberOfMessages << " read size: " << std::to_string(queue.usedSize));
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
  ((unsigned*)queue.buf)[-2] = static_cast<unsigned>(queue.usedSize);
  ((unsigned*)queue.buf)[-1] = (queue.numberOfMessages & 0x0fffffff) | (static_cast<unsigned>(queue.usedSize >> 4) & 0xf0000000);
  return queue.buf - MessageQueueBase::queueHeaderSize;
}
