/**
 * @file InMessage.cpp
 *
 * Implementation of class InMessageQueue, InBinaryMessage, InTextMessage,
 * and InMessage.
 *
 * @author Martin Lötzsch
 */

#include "InMessage.h"
#include "MessageQueue.h"

bool InMessageQueue::exists() const
{
  return true;
}

bool InMessageQueue::getEof() const
{
  return queue != nullptr && queue->eof();
}

void InMessageQueue::open(MessageQueueBase* q)
{
  if(queue == nullptr)
    queue = q;
}

void InMessageQueue::readFromStream(void* p, size_t size)
{
  if(queue != nullptr)
    queue->read(p, size);
}

InBinaryMessage::InBinaryMessage(MessageQueueBase* q)
{
  open(q);
}

InTextMessage::InTextMessage(MessageQueueBase* q)
{
  open(q);
}

std::string InTextMessage::readAll()
{
  std::string result,
              s;
  while(!eof())
  {
    *this >> s;
    result += s;
  }
  return result;
}

InMessage::InMessage(MessageQueueBase& queue) :
  queue(queue), bin(&queue), text(&queue)
{}

MessageID InMessage::getMessageID() const
{
  return queue.getMessageID();
}

int InMessage::getMessageSize() const
{
  return queue.getMessageSize();
}

int InMessage::getBytesLeft() const
{
  return queue.getBytesLeftInMessage();
}

void InMessage::resetReadPosition()
{
  queue.resetReadPosition();
  text.reset();
}

const char* InMessage::getData() const
{
  return queue.getData();
}
