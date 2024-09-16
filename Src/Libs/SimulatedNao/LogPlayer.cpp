/**
 * @file LogPlayer.cpp
 *
 * This file implements a log player that records or plays back log files (see
 * header for details).
 *
 * @author Thomas Röfer
 */

#include "LogPlayer.h"
#include "Framework/LoggingTools.h"
#include "Framework/Settings.h"
#include "Platform/File.h"
#include "Streaming/Global.h"
#include <filesystem>
#include <snappy-c.h>
#ifdef WINDOWS
#include <io.h>
#else
#include <unistd.h>
#endif

void LogPlayer::readMessageIDs(In& stream)
{
  std::unordered_map<std::string, MessageID> mapNameToID;
  FOREACH_ENUM(MessageID, id, numOfDataMessageIDs)
    mapNameToID[TypeRegistry::getEnumName(id)] = id;
  mapNameToID["idProcessBegin"] = idFrameBegin;
  mapNameToID["idProcessFinished"] = idFrameFinished;

  mapLogToID.clear();
  mapIDToLog.clear();
  logIDNames.clear();
  unsigned char size;
  stream >> size;
  mapLogToID.resize(size);
  mapIDToLog.resize(numOfDataMessageIDs);
  logIDNames.resize(size);
  for(unsigned id = 0; id < size; ++id)
  {
    stream >> logIDNames[id];
    auto i = mapNameToID.find(logIDNames[id]);
    if(i != mapNameToID.end())
    {
      mapLogToID[id] = i->second;
      mapIDToLog[i->second] = static_cast<MessageID>(id);
    }
    else
      mapLogToID[id] = undefined;
  }
}

void LogPlayer::writeMessageIDs(Out& stream)
{
  stream << static_cast<unsigned char>(logIDNames.size());
  for(const std::string& name : logIDNames)
    stream << name;
}

void LogPlayer::updateIndices()
{
  frameIndex.clear();
  framesHaveImage.clear();
  statsPerThread.clear();
  annotationsPerThread.clear();

  size_t frame = 0;
  const_iterator lastFrame = begin();
  bool hasImage = anyFrameHasImage = false;
  std::string currentThread;
  for(auto i = begin(); i != end(); ++i)
  {
    auto message = *i;
    switch(id(message))
    {
      case idFrameBegin:
        frame = i - begin();
        (*i).bin() >> currentThread;
        hasImage = false;
        break;
      case idFrameFinished:
        ASSERT(frameIndex.empty() || frameIndex.back() != frame);
        frameIndex.push_back(frame);
        framesHaveImage.push_back(hasImage);
        lastFrame = i;
        break;
      case idCameraImage:
      case idJPEGImage:
        hasImage = anyFrameHasImage = true;
        break;
      case idAnnotation:
      {
        AnnotationInfo::AnnotationData& annotation = annotationsPerThread[currentThread].emplace_back();
        annotation.read(*i);
        annotation.frame = static_cast<unsigned>(frameIndex.size());
      }
    }

    auto j = statsPerThread.find(currentThread);
    if(j == statsPerThread.end())
      j = statsPerThread.insert({currentThread, std::vector<std::pair<size_t, size_t>>(mapLogToID.size())}).first;
    auto& stat = j->second[message.id()];
    ++stat.first;
    stat.second +=  sizeof(MessageHeader) + message.size();
  }

  // If the last frame is not complete, drop partial information.
  resize(lastFrame == 0 ? 0 : ++lastFrame - begin());

  for(auto& [_, annotations] : annotationsPerThread)
    while(!annotations.empty() && annotations.back().frame == frameIndex.size())
      annotations.pop_back();

  sizeWhenIndexWasComputed = size();
}

bool LogPlayer::readIndices(In& stream, size_t& usedSize)
{
  unsigned char chunk;
  unsigned char version;
  stream >> chunk >> version;
  if(chunk != LoggingTools::logFileIndices || version != indexVersion)
    return false;

  stream >> reinterpret_cast<unsigned*>(&usedSize)[0] >> reinterpret_cast<unsigned*>(&usedSize)[1];

  unsigned size = 0;
  stream >> size;
  std::vector<size_t> offsets(size);
  stream.read(offsets.data(), offsets.size() * sizeof(offsets[0]));

  frameIndex.clear();
  framesHaveImage.clear();
  anyFrameHasImage = false;
  frameIndex.reserve(size);
  framesHaveImage.reserve(size);
  for(size_t offset : offsets)
  {
    frameIndex.push_back(offset & ~(1ull << 63));
    anyFrameHasImage |= framesHaveImage.emplace_back((offset & 1ull << 63) != 0);
  }

  statsPerThread.clear();
  stream >> size;
  for(unsigned i = 0; i < size; ++i)
  {
    std::string threadName;
    unsigned statsSize;
    stream >> threadName >> statsSize;
    statsPerThread[threadName].resize(statsSize);
    stream.read(statsPerThread[threadName].data(), statsSize * sizeof(statsPerThread[threadName][0]));
  }

  annotationsPerThread.clear();
  stream >> size;
  annotationsPerThread.reserve(size);
  for(unsigned i = 0; i < size; ++i)
  {
    std::string threadName;
    unsigned annotationsSize;
    stream >> threadName >> annotationsSize;
    std::vector<AnnotationInfo::AnnotationData>& annotations = annotationsPerThread[threadName];
    for(unsigned j = 0; j < annotationsSize; ++j)
    {
      AnnotationInfo::AnnotationData& annotation = annotations.emplace_back();
      stream >> annotation.annotationNumber >> annotation.frame >> annotation.name >> annotation.annotation;
    }
  }

  sizeWhenIndexWasComputed = usedSize;

  return true;
}

void LogPlayer::writeIndices(Out& stream) const
{
  if(sizeWhenIndexWasComputed != size())
    const_cast<LogPlayer*>(this)->updateIndices();

  stream << static_cast<unsigned char>(LoggingTools::logFileIndices) << indexVersion;

  stream << static_cast<unsigned>(size()) << static_cast<unsigned>(size() >> 32);

  stream << static_cast<unsigned>(frameIndex.size());
  std::vector<size_t> offsets;
  offsets.reserve(frameIndex.size());
  for(size_t i = 0; i < frameIndex.size(); ++i)
    offsets.push_back(frameIndex[i] | (framesHaveImage[i] ? 1ull << 63 : 0));
  stream.write(offsets.data(), offsets.size() * sizeof(offsets[0]));

  stream << static_cast<unsigned>(statsPerThread.size());
  for(const auto& [threadName, stats] : statsPerThread)
  {
    stream << threadName;
    stream << static_cast<unsigned>(stats.size());
    stream.write(stats.data(), stats.size() * sizeof(stats[0]));
  }

  stream << static_cast<unsigned>(annotationsPerThread.size());
  for(const auto& [threadName, annotations] : annotationsPerThread)
  {
    stream << threadName << static_cast<unsigned>(annotations.size());
    for(const AnnotationInfo::AnnotationData& annotation : annotations)
      stream << annotation.annotationNumber << annotation.frame << annotation.name << annotation.annotation;
  }
}

std::pair<size_t, size_t> LogPlayer::statOf(MessageID id, const std::string& threadName) const
{
  if(sizeWhenIndexWasComputed != size())
    const_cast<LogPlayer*>(this)->updateIndices();

  const MessageID logId = mapIDToLog[id];
  if(threadName.empty())
  {
    std::pair<size_t, size_t> sums = {0, 0};
    for(const auto& [_, stats] : statsPerThread)
    {
      sums.first += stats[logId].first;
      sums.second += stats[logId].second;
    }
    return sums;
  }
  else
  {
    auto i = statsPerThread.find(threadName);
    return i != statsPerThread.end() ? i->second[logId] : std::pair<size_t, size_t>();
  }
}

void LogPlayer::clear()
{
  MessageQueue::clear();
  path = "";
  file = nullptr;
  mapLogToID.clear();
  logIDNames.clear();
  mapLogToID.reserve(numOfDataMessageIDs);
  logIDNames.reserve(numOfDataMessageIDs);
  FOREACH_ENUM(MessageID, id, numOfDataMessageIDs)
  {
    mapLogToID.push_back(id);
    logIDNames.push_back(TypeRegistry::getEnumName(id));
  }
  mapIDToLog = mapLogToID;
  typeInfo = nullptr;
  frameIndex.clear();
  framesHaveImage.clear();
  statsPerThread.clear();
  annotationsPerThread.clear();
  currentFrame = 0;
  sizeWhenIndexWasComputed = 0;
}

bool LogPlayer::open(const std::string& fileName)
{
  std::unique_ptr<MemoryMappedFile> file = std::make_unique<MemoryMappedFile>(fileName);

  if(file->exists())
  {
    typeInfo = nullptr;
    InBinaryMemory stream(file->getData(), file->getSize());
    path = std::filesystem::absolute(File::isAbsolute(fileName)
                                     ? fileName
                                     : File::getBHDir() + ("/Config/" + fileName)).generic_string();
    for(;;)
    {
      char chunk;
      stream >> chunk;
      switch(chunk)
      {
        case LoggingTools::logFileSettings:
          LoggingTools::skipSettings(stream);
          break;
        case LoggingTools::logFileMessageIDs:
          readMessageIDs(stream);
          break;
        case LoggingTools::logFileTypeInfo:
          typeInfo = std::make_unique<TypeInfo>(false);
          stream >> *typeInfo;
          break;
        case LoggingTools::logFileCompressed: //compressed log file
          while(!stream.eof())
          {
            unsigned compressedSize;
            stream >> compressedSize;
            std::vector<char> compressedBuffer;
            compressedBuffer.resize(compressedSize);
            stream.read(compressedBuffer.data(), compressedSize);
            size_t uncompressedSize = 0;
            snappy_uncompressed_length(compressedBuffer.data(), compressedSize, &uncompressedSize);
            std::vector<char> uncompressBuffer;
            uncompressBuffer.resize(uncompressedSize);
            if(snappy_uncompress(compressedBuffer.data(), compressedSize, &uncompressBuffer[0], &uncompressedSize) != SNAPPY_OK)
              break;
            InBinaryMemory(uncompressBuffer.data(), uncompressedSize) >> *this;
          }
          updateIndices();
          return true;
        case LoggingTools::logFileUncompressed:
        {
          QueueHeader header;
          stream.read(&header, sizeof(QueueHeader));
          size_t usedSize = header.sizeLow | static_cast<size_t>(header.sizeHigh) << 32;
          const size_t position = stream.getPosition();
          size_t remainingSize = stream.getSize() - position;
          bool hasIndex = header.messages != 0x0fffffff && usedSize != remainingSize;
          if(hasIndex)
          {
            stream.skip(usedSize);
            if(!readIndices(stream, usedSize)) // Wrong index version -> remove index from file.
            {
              file = nullptr;
              {
                File f(fileName, "rb+");
#ifdef WINDOWS
                _chsize_s(_fileno(static_cast<FILE*>(f.getNativeFile())), position + usedSize);
#else
                ftruncate(fileno(static_cast<FILE*>(f.getNativeFile())), position + usedSize);
#endif
              }
              remainingSize = usedSize;
              hasIndex = false;
              file = std::make_unique<MemoryMappedFile>(fileName);
            }
          }
          if(!hasIndex) // No index -> create one and append it to file.
          {
            if(header.messages == 0x0fffffff)
              usedSize = remainingSize;

            setBuffer(file->getData() + position, usedSize);
            updateIndices();
            file = nullptr; // Close file

            // Append indices
            {
              OutBinaryFile stream(path, true);
              if(stream.exists())
              {
                stream.getFile()->skip(static_cast<ptrdiff_t>(position - 8));
                QueueHeader header = {static_cast<unsigned>(usedSize), 0, static_cast<unsigned>(usedSize >> 32)};
                stream.write(&header, sizeof(QueueHeader));
                stream.getFile()->skip(usedSize);
                writeIndices(stream);
              }
            }

            file = std::make_unique<MemoryMappedFile>(fileName); // reopen file
          }
          setBuffer(file->getData() + position, usedSize);
          this->file = std::move(file);
          return true;
        }
        default:
          path = "";
          return false; // unknown chunk
      }
    }
  }
  return false;
}

bool LogPlayer::save(std::string fileName, const TypeInfo& typeInfo)
{
  if(fileName.empty())
    fileName = path;

  fileName = std::filesystem::absolute(File::isAbsolute(fileName)
                                       ? fileName
                                       : File::getBHDir() + ("/Config/" + fileName)).generic_string();

  if(fileName == path && file != nullptr)
    return false;

  size_t index = fileName.find_last_of("\\/");
  if(index != std::string::npos)
    std::filesystem::create_directories(fileName.substr(0, index));

  OutBinaryFile file(fileName);
  if(file.exists())
  {
    path = fileName;
    file << LoggingTools::logFileSettings;
    LoggingTools::writeSettings(file, Global::getSettings());
    file << LoggingTools::logFileMessageIDs;
    writeMessageIDs(file);
    file << LoggingTools::logFileTypeInfo << (this->typeInfo ? *this->typeInfo : typeInfo);
    file << LoggingTools::logFileUncompressed << *this;
    writeIndices(file);
    return true;
  }
  else
    return false;
}

MessageID LogPlayer::id(Message message) const
{
  MessageID logId = message.id();
  return logId < mapLogToID.size() ? mapLogToID[logId] : undefined;
}

void LogPlayer::filter(const std::function<bool(const_iterator)>& keep)
{
  MessageQueue::filter(keep);
  updateIndices();
  currentFrame = -1;
  file = nullptr;
}

const std::unordered_map<std::string, std::vector<AnnotationInfo::AnnotationData>>& LogPlayer::annotations() const
{
  if(sizeWhenIndexWasComputed != size())
    const_cast<LogPlayer*>(this)->updateIndices();

  return annotationsPerThread;
}

void LogPlayer::playBack(size_t frame)
{
  if(!frameIndex.empty())
  {
    if(typeInfoRequested && typeInfo)
    {
      target.bin(idTypeInfo) << *typeInfo;
      typeInfoRequested = false;
    }
    if(static_cast<ptrdiff_t>(frame) < 0)
    {
      currentFrame = -1;
      return;
    }
    else if(frame >= frameIndex.size())
    {
      frame = cycle ? frame % frameIndex.size() : frameIndex.size() - 1;
      if(currentFrame == frame)
        return;
    }
    size_t originalSize = target.size();
    const_iterator end = frame + 1 == frameIndex.size() ? this->end() : begin() + frameIndex[frame + 1];
    target << std::pair<const_iterator, const_iterator>(begin() + frameIndex[frame], end);
    for(auto i = target.begin() + originalSize; i != target.end(); ++i)
    {
      const MessageID id = static_cast<MessageID>(*i.current);
      *const_cast<char*>(i.current) = id < mapLogToID.size() ? mapLogToID[id] : undefined;
    }
  }
  currentFrame = frame;
}

size_t LogPlayer::nextImageFrame(size_t frame) const
{
  if(anyFrameHasImage)
    do
    {
      if(cycle)
        frame = (frame + 1) % frameIndex.size();
      else if(frame + 1 < frameIndex.size())
        ++frame;
      else
        break;
    }
    while(!framesHaveImage[frame]);
  return frame;
}

size_t LogPlayer::prevImageFrame(size_t frame) const
{
  if(anyFrameHasImage && frame < frameIndex.size())
    while(frame > 0 && !framesHaveImage[--frame]);
  return frame;
}

std::string LogPlayer::threadOf(size_t frame) const
{
  std::string thread;
  if(!frameIndex.empty())
  {
    if(static_cast<ptrdiff_t>(frame) < 0)
      frame = 0;
    else if(frame >= frameIndex.size())
      frame = cycle ? frame % frameIndex.size() : frameIndex.size() - 1;
    (*(begin() + frameIndex[frame])).bin() >> thread;
  }
  return thread;
}
