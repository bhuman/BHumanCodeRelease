/**
 * @file MessageQueueBase.cpp
 * Implementation of the class that performs the memory management for the class MessageQueue.
 * @author Martin Lötzsch
 * @author Thomas Röfer
 */

#include "MessageQueueBase.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/InOut.h"
#include "Tools/Streams/InStreams.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <unordered_map>

MessageQueueBase::MessageQueueBase()
#ifndef TARGET_ROBOT
  :
  maximumSize(0x4000000), // 64 MB
  reservedSize(16384)
{
  buf = static_cast<char*>(malloc(reservedSize + queueHeaderSize));
  ASSERT(buf);
  buf += queueHeaderSize;
#else
{
#endif
}

MessageQueueBase::~MessageQueueBase()
{
  freeIndex();
  if(buf)
    free(buf - queueHeaderSize);
  if(mappedIDs)
  {
    delete[] mappedIDs;
    delete[] mappedIDNames;
  }
}

void MessageQueueBase::setSize(size_t size, size_t reserveForInfrastructure)
{
  this->reserveForInfrastructure = reserveForInfrastructure;
  size = std::min(std::numeric_limits<size_t>::max() - queueHeaderSize, size);
#ifdef TARGET_ROBOT
  ASSERT(!buf);
  buf = static_cast<char*>(malloc(size + queueHeaderSize));
  ASSERT(buf);
  buf += queueHeaderSize;
#else
  ASSERT(size >= usedSize);
  if(size < reservedSize)
  {
    char* newBuf = static_cast<char*>(realloc(buf - queueHeaderSize, size + queueHeaderSize));
    if(newBuf)
    {
      buf = newBuf + queueHeaderSize;
      reservedSize = size;
    }
  }
#endif
  maximumSize = size;
}

void MessageQueueBase::clear()
{
  usedSize = 0;
  numberOfMessages = 0;
  writePosition = 0;
  writingOfLastMessageFailed = false;
  selectedMessageForReadingPosition = 0;
  readPosition = 0;
  lastMessage = 0;
  numOfMappedIDs = 0;
  if(mappedIDs)
  {
    delete[] mappedIDs;
    mappedIDs = 0;
    delete[] mappedIDNames;
  }
  freeIndex();
}

void MessageQueueBase::createIndex()
{
  freeIndex();
  messageIndex = new size_t[numberOfMessages];
  selectedMessageForReadingPosition = 0;
  for(int i = 0; i < numberOfMessages; ++i)
  {
    messageIndex[i] = selectedMessageForReadingPosition;
    selectedMessageForReadingPosition += getMessageSize() + headerSize;
  }
}

void MessageQueueBase::freeIndex()
{
  if(messageIndex)
  {
    delete[] messageIndex;
    messageIndex = 0;
  }
}

void MessageQueueBase::removeMessage(int message)
{
  freeIndex();
  selectedMessageForReadingPosition = 0;
  int i;
  for(i = 0; i < message; ++i)
    selectedMessageForReadingPosition += getMessageSize() + headerSize;
  usedSize = selectedMessageForReadingPosition;
  for(++i; i < numberOfMessages; ++i)
  {
    int mlength = getMessageSize() + headerSize;
    selectedMessageForReadingPosition += mlength;
    memcpy(buf + usedSize, buf + selectedMessageForReadingPosition, mlength);
    usedSize = selectedMessageForReadingPosition;
  }
  readPosition = 0;
  --numberOfMessages;
  selectedMessageForReadingPosition = 0;
  lastMessage = 0;
}

char* MessageQueueBase::reserve(size_t size)
{
  size_t currentSize = usedSize + headerSize + writePosition;
  if(static_cast<unsigned long long>(currentSize + size) > static_cast<unsigned long long>(maximumSize))
    return nullptr;
  else
  {
#ifndef TARGET_ROBOT
    size_t r = reservedSize;
    if(currentSize + size >= r)
    {
      r *= 2;
      if(currentSize + size >= r)
        r = (currentSize + size) * 4;
    }
    if(r > maximumSize)
      r = maximumSize;
    if(r > reservedSize)
    {
      char* newBuf = static_cast<char*>(realloc(buf - queueHeaderSize, r + queueHeaderSize));
      if(newBuf)
      {
        buf = newBuf + queueHeaderSize;
        reservedSize = r;
      }
      else
      {
        maximumSize = reservedSize;
        return nullptr;
      }
    }
#endif
    writePosition += static_cast<unsigned>(size);
    return buf + currentSize;
  }
}

void MessageQueueBase::write(const void* p, size_t size)
{
  ASSERT(!messageIndex);
  if(!writingOfLastMessageFailed)
  {
    char* dest = reserve(size);
    if(dest)
      memcpy(dest, p, size);
    else
      writingOfLastMessageFailed = true;
  }
}

bool MessageQueueBase::finishMessage(MessageID id)
{
  ASSERT(buf);
  ASSERT(!messageIndex);
  bool success = !writingOfLastMessageFailed;
  if(success)
  {
    if(reserveForInfrastructure > maximumSize - usedSize - writePosition - headerSize)
      switch(id)
      {
        // When these messages are lost, communication might get stuck
        case idFrameBegin:
        case idFrameFinished:
        case idDebugRequest:
        case idDebugResponse:
        case idDebugDataResponse:
        case idDebugDataChangeRequest:
        case idTypeInfo:
        case idModuleTable:
        case idModuleRequest:
        case idLogResponse:
        case idDrawingManager:
        case idDrawingManager3D:
        case idConsole:
        case idRobotname:
        case idFieldColors:
        case idAudioData: // continuous data stream required
          break; // accept
        default:
          success = false; // reject
      }

    if(success)
    {
      memcpy(buf + usedSize, reinterpret_cast<char*>(&id), 1); // write the id of the message
      memcpy(buf + usedSize + 1, &writePosition, 3); // write the size of the message
      ++numberOfMessages;
      usedSize += writePosition + headerSize;
    }
  }

  writePosition = 0;
  writingOfLastMessageFailed = false;

  return success;
}

void MessageQueueBase::removeRepetitions()
{
  ASSERT(!messageIndex);
  std::unordered_map<std::string, unsigned short[numOfMessageIDs]> threads;
  // if messages sent before the first idFrameBegin
  const std::string unknown("unknown");
  unsigned short* messagesPerType =  threads[unknown];

  selectedMessageForReadingPosition = 0;

  // streamed strings have 4 Bytes before the real string
  constexpr int offset = 4;
  for(int i = 0; i < numberOfMessages; ++i)
  {
    if(getMessageID() == idFrameBegin)
    {
      ASSERT(offset < getMessageSize());
      // is added and initialized with 0 if not present
      messagesPerType = threads[std::string(getData() + offset, getMessageSize() - offset)];
    }
    ++messagesPerType[getMessageID()];
    selectedMessageForReadingPosition += getMessageSize() + headerSize;
  }

  // reset all variables
  selectedMessageForReadingPosition = 0;
  usedSize = 0;
  int numOfDeleted = 0;
  size_t frameBegin = std::numeric_limits<size_t>::max();
  bool frameEmpty = true;
  messagesPerType = threads[unknown];

  for(int i = 0; i < numberOfMessages; ++i)
  {
    int mlength = getMessageSize() + headerSize;
    bool copy;
    switch(getMessageID())
    {
      // accept up to 20 times, thread id is not important
      case idText:
        copy = --messagesPerType[idText] <= 20;
        break;

      // accept always, thread id is not important
      case idDebugRequest:
      case idDebugResponse:
      case idDebugDataResponse:
      case idPlot:
      case idConsole:
      case idAudioData:
      case idAnnotation:
      case idLogResponse:
        copy = true;
        break;

      // data only from latest frame
      case idStopwatch:
      case idDebugImage:
      case idDebugDrawing:
      case idDebugDrawing3D:
        copy = messagesPerType[idFrameFinished] == 1;
        break;

      // always accept, but may be reverted later
      case idFrameBegin:
        if(frameBegin != std::numeric_limits<size_t>::max()) // nothing between last idFrameBegin and this one, so remove idFrameBegin as well
        {
          usedSize = frameBegin;
          ++numOfDeleted;
        }
        messagesPerType = threads[std::string(getData() + offset, getMessageSize() - offset)];
        copy = true;
        break;

      case idFrameFinished:
        ASSERT(messagesPerType == threads[std::string(getData() + offset, getMessageSize() - offset)]);
        copy = !frameEmpty; // nothing since last idFrameBegin or idFrameFinished, no new idFrameFinished required
        --messagesPerType[idFrameFinished];
        break;

      default:
        if(getMessageID() < numOfDataMessageIDs && getMessageID() != idFieldColors) // data only from latest frame
          copy = messagesPerType[idFrameFinished] == 1;
        else // only the latest other messages
          copy = --messagesPerType[getMessageID()] == 0;
    }

    if(copy)
    {
      // Remember position of begin of frame, but forget it, when another message was copied.
      // So idFrameBegin idFrameFinished will be removed.
      if(getMessageID() == idFrameBegin) // remember begin of frame
      {
        frameBegin = usedSize;
        frameEmpty = true; // assume next frame as empty
      }
      else if(getMessageID() == idFrameFinished)
        frameEmpty = true; // assume next frame as empty
      else // we copy a message within a frame so the idFrameBegin/Finished must stay
      {
        frameBegin = std::numeric_limits<size_t>::max();
        frameEmpty = false;
      }

      //this message is important, it shall be copied
      if(usedSize != selectedMessageForReadingPosition)
        memmove(buf + usedSize, buf + selectedMessageForReadingPosition, mlength);
      usedSize += mlength;
    }
    else
      ++numOfDeleted;
    selectedMessageForReadingPosition += mlength;
  }
  numberOfMessages -= numOfDeleted;
  readPosition = 0;
  selectedMessageForReadingPosition = 0;
  lastMessage = 0;
}

MessageID MessageQueueBase::getMessageID() const
{
  MessageID id = static_cast<MessageID>(buf[selectedMessageForReadingPosition]);
  return id < numOfMappedIDs ? mappedIDs[id] : id;
}

void MessageQueueBase::setSelectedMessageForReading(int message)
{
  ASSERT(message >= 0);
  ASSERT(message < numberOfMessages);

  if(messageIndex)
    selectedMessageForReadingPosition = messageIndex[message];
  else
  {
    int m = message;
    if(m >= lastMessage)
    {
      ASSERT(lastMessage < numberOfMessages);
      m -= lastMessage;
    }
    else
      selectedMessageForReadingPosition = 0;

    for(int i = 0; i < m; ++i)
      selectedMessageForReadingPosition += getMessageSize() + headerSize;
  }

  readPosition = 0;
  lastMessage = message;
}

void MessageQueueBase::read(void* p, size_t size)
{
  ASSERT(readPosition + static_cast<int>(size) <= getMessageSize());
  memcpy(p, buf + selectedMessageForReadingPosition + headerSize + readPosition, size);
  readPosition += static_cast<int>(size);
}

void MessageQueueBase::writeMessageIDs(Out& stream, MessageID numOfMessageIDs) const
{
  if(mappedIDs)
  {
    stream << numOfMappedIDs;
    for(int i = 0; i < numOfMappedIDs; ++i)
      stream << mappedIDNames[i];
  }
  else
  {
    stream << static_cast<unsigned char>(numOfMessageIDs);
    FOREACH_ENUM(MessageID, i, numOfMessageIDs)
      stream << TypeRegistry::getEnumName(i);
  }
}

void MessageQueueBase::readMessageIDMapping(In& stream)
{
  ASSERT(!mappedIDs);
  stream >> numOfMappedIDs;
  mappedIDs = new MessageID[numOfMappedIDs];
  mappedIDNames = new std::string[numOfMappedIDs];
  memset(mappedIDs, undefined, numOfMappedIDs);

  for(int i = 0; i < numOfMappedIDs; ++i)
  {
    stream >> mappedIDNames[i];
    FOREACH_ENUM(MessageID, j)
    {
      if(mappedIDNames[i] == TypeRegistry::getEnumName(j))
      {
        mappedIDs[i] = j;
        break;
      }
      // HACK for old logs: Map message id and activate conversion.
      else if(mappedIDNames[i] == "idProcessBegin")
      {
        mappedIDs[i] = MessageID::idFrameBegin;
        break;
      }
      else if(mappedIDNames[i] == "idProcessFinished")
      {
        mappedIDs[i] = MessageID::idFrameFinished;
        break;
      }
    }
  }
}
