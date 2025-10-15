/**
 * @file Log.cpp
 *
 * This file implements a class that represents a log file in memory.
 * It supports iterating through the log file frame by frame and
 * accessing the representations that are stored in them. Message ids
 * are automatically mapped to the current ones. If the type information
 * contained in the log file differs from the current one,
 * representations are converted.
 *
 * @author Thomas Röfer
 */

#include "Log.h"
#include "Debugging/DebugDataStreamer.h"
#include "../../Representations/Infrastructure/AudioData.h"

const Log::Message& Log::Frame::operator[](const MessageID id) const
{
  auto i = messages.find(id);
  ASSERT(i != messages.end());
  return i->second;
}

Log::Message::~Message()
{
  if(representation)
  {
    switch(id())
    {
      case idFrameBegin:
      case idFrameFinished:
        delete reinterpret_cast<std::string*&>(representation);
        break;
      default:
        delete representation;
    }
  }
}

void Log::Message::fillRepresentation() const
{
  ASSERT(log->logPlayer->typeInfo);
  const MessageID id = this->id();
  for(;;)
    switch(log->states[id])
    {
      case unknown:
      {
        // Check whether the current and the logged specifications are the same.
        const char* type = TypeRegistry::getEnumName(id) + 2;
        const_cast<Log*>(log)->states[id] = TypeInfo::current->areTypesEqual(*log->logPlayer->typeInfo, type, type) ? accept : convert;
        continue;
      }
      case accept:
        bin() >> *representation;
        return;
      case convert:
      {
        const char* type = TypeRegistry::getEnumName(id) + 2;

        // Stream into textual representation in memory using type specification of log file.
        OutMapMemory outMap(true, 16384);
        auto stream = bin();
        DebugDataStreamer streamer(*log->logPlayer->typeInfo, stream, type);
        outMap << streamer;

        // Read from textual representation. Errors are suppressed.
        InMapMemory inMap(outMap.data(), outMap.size(), 0);
        inMap >> *representation;

        // HACK: This does not work if anything else than the sample format is changed in AudioData.
        if(id == idAudioData)
        {
          AudioData& audioData = static_cast<AudioData&>(*representation);
          for(AudioData::Sample& sample : audioData.samples)
            sample /= std::numeric_limits<short>::max();
        }
        return;
      }
    }
}

Log::Message::operator const std::string&() const
{
  ASSERT(id() == idFrameBegin || id() == idFrameFinished);
  std::string*& representation = reinterpret_cast<std::string*&>(this->representation);
  if(!representation)
  {
    representation = new std::string;
    bin() >> *representation;
  }
  return *representation;
}

Log::Frame::Frame(size_t frame, const Log* log)
{
  const LogPlayer* logPlayer = log->logPlayer;
  const MessageQueue::const_iterator end = frame + 1 < logPlayer->frames()
                                           ? logPlayer->begin() + logPlayer->frameIndex[frame + 1]
                                           : logPlayer->end();
  for(MessageQueue::const_iterator i = logPlayer->begin() + logPlayer->frameIndex[frame]; i != end; ++i)
  {
    const Message message(*i, log);
    const MessageID id = message.id();
    if(id == idAnnotation)
      annotationMessages.push_back(message);
    else
      messages.emplace(id, message);
  }
}

bool Log::Frame::contains(const MessageID id) const
{
  return messages.contains(id);
}

static MessageQueue dummy;

Log::Log(const std::string& filename)
  : Log(*new LogPlayer(dummy), true)
{
  const_cast<LogPlayer*>(logPlayer)->open(filename);
}

Log::Log(const LogPlayer& logPlayer, const bool ownsLogPlayer)
  : logPlayer(&logPlayer),
    ownsLogPlayer(ownsLogPlayer)
{
  states.fill(unknown);
  states[idAnnotation] = states[idStopwatch] = accept;
  TypeInfo::initCurrent();
}

Log::~Log()
{
  if(ownsLogPlayer)
    delete logPlayer;
}
