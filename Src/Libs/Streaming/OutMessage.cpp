/**
 * @file OutMessage.cpp
 *
 * Implementation of class OutMessage, OutBinaryMessage, OutTextMessage,
 * OutConfigMessage and OutMessageQueue.
 *
 * @author Martin Lötzsch
 */

#include "OutMessage.h"
#include "MessageQueue.h"

void OutMessageQueue::open(MessageQueueBase* q)
{
  if(queue == nullptr)
    queue = q;
}

void OutMessageQueue::writeToStream(const void* p, size_t size)
{
  if(queue != nullptr)
    queue->write(p, size);
}

OutBinaryMessage::OutBinaryMessage(MessageQueueBase* q)
{
  open(q);
}

OutTextMessage::OutTextMessage(MessageQueueBase* q)
{
  open(q);
}

OutTextRawMessage::OutTextRawMessage(MessageQueueBase* q)
{
  open(q);
}

OutMessage::OutMessage(MessageQueueBase& queue) :
  queue(queue), bin(&queue), text(&queue), textRaw(&queue)
{}

bool OutMessage::finishMessage(MessageID id)
{
  return queue.finishMessage(id);
}

void OutMessage::cancelMessage()
{
  queue.cancelMessage();
}
