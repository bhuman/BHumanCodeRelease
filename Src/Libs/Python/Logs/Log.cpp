/**
 * @file Log.cpp
 *
 * This file implements the class representing a log file.
 *
 * @author Arne Hasselbring
 * @author Jan Fiedler
 */

#include "Log.h"
#include "Framework/LoggingTools.h"
#include "Platform/File.h"
#include "Streaming/InStreams.h"
#include <snappy-c.h>
#include <stdexcept>

Log::Log(const std::string& path, bool keepGoing) :
  typeInfo(false),
  keepGoing(keepGoing)
{
  std::unique_ptr<MemoryMappedFile> file = std::make_unique<MemoryMappedFile>(File::isAbsolute(path) ? path : "./" + path);
  if(!file->exists())
    throw std::runtime_error("Could not open log.");

  InBinaryMemory stream(file->getData(), file->getSize());

  LoggingTools::parseName(path.substr(path.find_last_of("/\\") + 1), &headName, &bodyName, &scenario, &location, &identifier, &playerNumber, &suffix);

  LoggingTools::LogFileFormat magicByte;
  stream >> magicByte;
  if(magicByte == LoggingTools::logFileSettings)
  {
    LoggingTools::skipSettings(stream);
    stream >> magicByte;
  }
  if(magicByte != LoggingTools::logFileMessageIDs)
    throw std::runtime_error("Log does not contain message ID section.");
  readMessageIDs(stream);
  stream >> magicByte;
  if(magicByte != LoggingTools::logFileTypeInfo)
    throw std::runtime_error("Log does not contain type info section.");
  stream >> typeInfo;
  stream >> magicByte;

  auto messageIDEnum = typeInfo.enums.find("MessageID");
  if(messageIDEnum == typeInfo.enums.end())
    throw std::runtime_error("Log type info does not contain MessageID.");
  messageIDNames = &messageIDEnum->second;

  // TODO: do not read everything into memory at once
  switch(magicByte)
  {
    case LoggingTools::logFileUncompressed:
    {
      QueueHeader header;
      stream.read(&header, sizeof(QueueHeader));
      size_t usedSize = header.sizeLow | static_cast<size_t>(header.sizeHigh) << 32;
      const size_t position = stream.getPosition();
      if(header.messages == 0x0fffffff)
        usedSize = stream.getSize() - position;
      setBuffer(file->getData() + position, usedSize);
      this->file = std::move(file);
      break;
    }
    case LoggingTools::logFileCompressed:
      reserve(0xfffffffffull);
      while(!stream.eof())
      {
        unsigned compressedSize;
        stream >> compressedSize;
        if(!compressedSize)
          throw std::runtime_error("Corrupt compressed log file.");
        std::vector<char> compressedBuffer(compressedSize);
        stream.read(compressedBuffer.data(), compressedSize);

        std::size_t uncompressedSize = 0;
        snappy_uncompressed_length(compressedBuffer.data(), compressedSize, &uncompressedSize);
        std::vector<char> uncompressedBuffer(uncompressedSize);
        if(snappy_uncompress(compressedBuffer.data(), compressedSize, uncompressedBuffer.data(), &uncompressedSize) != SNAPPY_OK)
          break;
        InBinaryMemory(uncompressedBuffer.data(), uncompressedSize) >> *this;
      }
      break;
    default:
      throw std::runtime_error("Unknown magic byte!");
  }

  // Calc numberOfFrames
  numberOfFrames = 0;
  for(Message message : *this)
    if(id(message) == idFrameBegin)
      ++numberOfFrames;
}

MessageID Log::id(Message message) const
{
  MessageID logId = message.id();
  return logId < mapLogToID.size() ? mapLogToID[logId] : undefined;
}

Frame Log::iter()
{
  return Frame(*this);
}

void Log::readMessageIDs(In& stream)
{
  std::unordered_map<std::string, MessageID> mapNameToID;
  FOREACH_ENUM(MessageID, id, numOfDataMessageIDs)
    mapNameToID[TypeRegistry::getEnumName(id)] = id;
  mapNameToID["idProcessBegin"] = idFrameBegin;
  mapNameToID["idProcessFinished"] = idFrameFinished;

  mapLogToID.clear();
  mapIDToLog.clear();
  unsigned char size;
  stream >> size;
  mapLogToID.resize(size);
  mapIDToLog.resize(numOfDataMessageIDs);
  for(unsigned id = 0; id < size; ++id)
  {
    std::string logIDName;
    stream >> logIDName;
    auto i = mapNameToID.find(logIDName);
    if(i != mapNameToID.end())
    {
      mapLogToID[id] = i->second;
      mapIDToLog[i->second] = static_cast<MessageID>(id);
    }
    else
      mapLogToID[id] = undefined;
  }
}
