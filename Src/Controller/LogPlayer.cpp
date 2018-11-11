/**
 * @file Controller/LogPlayer.cpp
 *
 * Implementation of class LogPlayer
 *
 * @author Martin Lötzsch
 */

#include <QImage>
#include <QFileInfo>
#include "LogPlayer.h"
#include "Platform/File.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/LabelImage.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Logging/LogFileFormat.h"

#include <snappy-c.h>

LogPlayer::LogPlayer(MessageQueue& targetQueue) :
  targetQueue(targetQueue)
{
  init();
}

void LogPlayer::init()
{
  clear();
  stop();
  numberOfFrames = 0;
  numberOfMessagesWithinCompleteFrames = 0;
  replayOffset = 0;
  state = initial;
  loop = false; //default: loop disabled
  typeInfo = nullptr;
  typeInfoReplayed = false;
  logfilePath = "";
}

bool LogPlayer::open(const std::string& fileName)
{
  InBinaryFile file(fileName);

  if(file.exists())
  {
    init();
    logfilePath = QFileInfo(fileName.c_str()).absoluteFilePath().toUtf8().constData();

    char magicByte;
    file >> magicByte;

    if(magicByte == Logging::logFileMessageIDs)
    {
      readMessageIDMapping(file);
      file >> magicByte;
    }

    if(magicByte == Logging::logFileTypeInfo)
    {
      typeInfo = std::unique_ptr<TypeInfo>(new TypeInfo(false));
      file >> *typeInfo;
      file >> magicByte;
    }

    switch(magicByte)
    {
      case Logging::logFileUncompressed: //regular log file
        file >> *this;
        break;
      case Logging::logFileCompressed: //compressed log file
        while(!file.eof())
        {
          unsigned compressedSize;
          file >> compressedSize;
          ASSERT(compressedSize > 0);
          std::vector<char> compressedBuffer;
          compressedBuffer.resize(compressedSize);
          file.read(&compressedBuffer[0], compressedSize);

          size_t uncompressedSize = 0;
          snappy_uncompressed_length(&compressedBuffer[0], compressedSize, &uncompressedSize);
          std::vector<char> uncompressBuffer;
          uncompressBuffer.resize(uncompressedSize);
          if(snappy_uncompress(&compressedBuffer[0], compressedSize, &uncompressBuffer[0], &uncompressedSize) != SNAPPY_OK)
            break;
          InBinaryMemory mem(&uncompressBuffer[0], uncompressedSize);
          mem >> *this;
        }
        break;
      default:
        logfilePath = "";
        return false; //unknown magic byte
    }

    stop();
    countFrames();
    createIndices();
    logfileMerged = false;
    loadLabels();
    return true;
  }
  return false;
}

void LogPlayer::play()
{
  state = playing;
}

void LogPlayer::stop()
{
  if(state == recording)
  {
    recordStop();
    return;
  }
  currentMessageNumber = -1;
  currentFrameNumber = -1;
  state = initial;
  lastImageFrameNumber = -1;
}

void LogPlayer::pause()
{
  if(getNumberOfMessages() == 0)
    state = initial;
  else
    state = paused;
}

void LogPlayer::stepBackward()
{
  pause();
  if(state == paused)
  {
    if(currentFrameNumber > 0)
      --currentFrameNumber;
    else if(loop && numberOfFrames > 0)
      currentFrameNumber = numberOfFrames - 1;
    else
      return;
    ASSERT(currentFrameNumber < static_cast<int>(frameIndex.size()));
    currentMessageNumber = frameIndex[currentFrameNumber];

    queue.setSelectedMessageForReading(currentMessageNumber);
    stepRepeat();
  }
}

void LogPlayer::stepImageBackward()
{
  pause();
  if(state == paused && (currentFrameNumber > 0 || (loop && numberOfFrames > 0)))
  {
    int lastImageFrameNumber = this->lastImageFrameNumber;
    int thisFrameNumber = currentFrameNumber;
    do
      stepBackward();
    while(lastImageFrameNumber == this->lastImageFrameNumber
          && thisFrameNumber != currentFrameNumber
          && (currentFrameNumber > 0 || (loop && numberOfFrames > 0)));
  }
}

void LogPlayer::stepForward()
{
  pause();
  if(state == paused)
  {
    if(currentFrameNumber >= numberOfFrames - 1 || currentMessageNumber >= numberOfMessagesWithinCompleteFrames - 1)
    {
      if(loop && numberOfFrames > 0)
      {
        currentMessageNumber = -1;
        currentFrameNumber = -1;
      }
      else
        return;
    }

    replayTypeInfo();

    do
    {
      copyMessage(++currentMessageNumber, targetQueue);
      if(queue.getMessageID() == idImage
         || queue.getMessageID() == idJPEGImage
         || queue.getMessageID() == idThumbnail
         || queue.getMessageID() == idImagePatches
         || (queue.getMessageID() == idLowFrameRateImage && queue.getMessageSize() > 1000))
        lastImageFrameNumber = currentFrameNumber + 1;
    }
    while(queue.getMessageID() != idProcessFinished);

    ++currentFrameNumber;
  }
}

void LogPlayer::stepImageForward()
{
  pause();
  if(state == paused && (currentFrameNumber < numberOfFrames - 1 || (loop && numberOfFrames > 0)))
  {
    int lastImageFrameNumber = this->lastImageFrameNumber;
    int thisFrameNumber = currentFrameNumber;
    do
      stepForward();
    while(lastImageFrameNumber == this->lastImageFrameNumber
          && thisFrameNumber != currentFrameNumber
          && (currentFrameNumber < numberOfFrames - 1 || (loop && numberOfFrames > 0)));
  }
}

void LogPlayer::stepRepeat()
{
  pause();
  if(state == paused && currentFrameNumber >= 0)
  {
    --currentFrameNumber;
    currentMessageNumber = currentFrameNumber >= 0 ? frameIndex[currentFrameNumber] - 1 : -1;
    stepForward();
  }
}

void LogPlayer::gotoFrame(int frame)
{
  pause();

  if(state == paused && frame < numberOfFrames)
  {
    currentFrameNumber = frame - 1;
    currentMessageNumber = currentFrameNumber >= 0 ? frameIndex[currentFrameNumber] - 1 : -1;
    stepForward();
  }
}

int LogPlayer::getFrameForRemainingGCTime(int time)
{
  return time < 0 || time >= static_cast<int>(gcTimeIndex.size()) ? -1 : gcTimeIndex[time];
}

void LogPlayer::recordStart()
{
  state = recording;
}

void LogPlayer::recordStop()
{
  while(getNumberOfMessages() > numberOfMessagesWithinCompleteFrames)
    removeLastMessage();
  currentMessageNumber = -1;
  currentFrameNumber = -1;
  state = initial;
}

void LogPlayer::setLoop(bool loop)
{
  this->loop = loop;
}

void LogPlayer::handleMessage(InMessage& message)
{
  if(state == recording)
  {
    message >> *this;
    if(message.getMessageID() == idProcessFinished)
    {
      numberOfMessagesWithinCompleteFrames = getNumberOfMessages();
      ++numberOfFrames;
    }
  }
}

bool LogPlayer::replay()
{
  if(state == playing)
  {
    if(currentFrameNumber < numberOfFrames - 1)
    {
      replayTypeInfo();

      do
      {
        copyMessage(++currentMessageNumber, targetQueue);
        if(queue.getMessageID() == idImage
           || queue.getMessageID() == idJPEGImage
           || queue.getMessageID() == idThumbnail
           || queue.getMessageID() == idImagePatches
           || (queue.getMessageID() == idLowFrameRateImage && queue.getMessageSize() > 1000))
          lastImageFrameNumber = currentFrameNumber + 1;
      }
      while(queue.getMessageID() != idProcessFinished && currentMessageNumber < numberOfMessagesWithinCompleteFrames - 1);

      ++currentFrameNumber;
      if(currentFrameNumber == numberOfFrames - 1)
      {
        if(loop)   //restart in loop mode
        {
          gotoFrame(0);
          play();
        }
      }
      return true;
    }
    else
    {
      if(loop)   //restart in loop mode
      {
        gotoFrame(0);
        play();
      }
    }
  }
  return false;
}

void LogPlayer::keep(const std::function<bool(InMessage&)>& filter)
{
  stop();
  LogPlayer temp((MessageQueue&)*this);
  temp.setSize(queue.getSize());
  moveAllMessages(temp);
  for(temp.currentMessageNumber = 0; temp.currentMessageNumber < temp.getNumberOfMessages(); ++temp.currentMessageNumber)
  {
    temp.queue.setSelectedMessageForReading(temp.currentMessageNumber);
    temp.in.text.reset();
    if(filter(temp.in))
      temp.copyMessage(temp.currentMessageNumber, *this);
  }
  countFrames();
  if(!frameIndex.empty())
    createIndices();
}

void LogPlayer::keepFrames(const std::function<bool(InMessage&)>& filter)
{
  stop();
  LogPlayer temp((MessageQueue&)*this);
  temp.setSize(queue.getSize());
  moveAllMessages(temp);
  temp.queue.createIndex();
  int frameStart = -1;
  bool keepFrame = false;
  for(temp.currentMessageNumber = 0; temp.currentMessageNumber < temp.getNumberOfMessages(); ++temp.currentMessageNumber)
  {
    temp.queue.setSelectedMessageForReading(temp.currentMessageNumber);
    temp.in.text.reset();
    keepFrame |= filter(temp.in);
    if(temp.queue.getMessageID() == idProcessBegin)
    {
      frameStart = temp.currentMessageNumber;
      keepFrame = false;
    }
    else if(temp.queue.getMessageID() == idProcessFinished && keepFrame)
    {
      int thisFrame = temp.currentMessageNumber;
      for(temp.currentMessageNumber = frameStart; temp.currentMessageNumber <= thisFrame; ++temp.currentMessageNumber)
        temp.copyMessage(temp.currentMessageNumber, *this);
      temp.currentMessageNumber = thisFrame;
      keepFrame = false;
    }
  }
  countFrames();
  if(!frameIndex.empty())
    createIndices();
}

void LogPlayer::keep(const std::vector<int>& messageNumbers)
{
  stop();
  LogPlayer temp((MessageQueue&)*this);
  temp.setSize(queue.getSize());
  moveAllMessages(temp);

  for(auto& messageNumber : messageNumbers)
  {
    temp.queue.setSelectedMessageForReading(messageNumber);
    temp.copyMessage(messageNumber, *this);
  }
  countFrames();
  if(!frameIndex.empty())
    createIndices();
}

void LogPlayer::statistics(int frequencies[numOfDataMessageIDs], unsigned* sizes, char processIdentifier)
{
  FOREACH_ENUM(MessageID, id, numOfDataMessageIDs)
    frequencies[id] = 0;
  if(sizes)
    FOREACH_ENUM(MessageID, id, numOfDataMessageIDs)
      sizes[id] = 0;

  if(getNumberOfMessages() > 0)
  {
    int current = queue.getSelectedMessageForReading();
    char currentProcess = 0;
    for(int i = 0; i < getNumberOfMessages(); ++i)
    {
      queue.setSelectedMessageForReading(i);
      ASSERT(queue.getMessageID() < numOfDataMessageIDs);
      if(queue.getMessageID() == idProcessBegin)
        currentProcess = queue.getData()[0];
      if(!processIdentifier || processIdentifier == currentProcess)
      {
        ++frequencies[queue.getMessageID()];
        if(sizes)
          sizes[queue.getMessageID()] += queue.getMessageSize() + 4;
      }
    }
    queue.setSelectedMessageForReading(current);
  }
}

void LogPlayer::createIndices()
{
  GameInfo gameInfo;
  OutBinaryMemory gameInfoSize(256);
  gameInfoSize << gameInfo;

  queue.createIndex();
  frameIndex.clear();
  frameIndex.reserve(numberOfFrames);
  gcTimeIndex.fill(-1);
  int frame = 0;
  for(int i = 0; i < getNumberOfMessages(); ++i)
  {
    queue.setSelectedMessageForReading(i);
    const MessageID id = queue.getMessageID();
    if(id == idProcessBegin)
      frameIndex.push_back(i);
    else if(id == idGameInfo && queue.getMessageSize() == static_cast<int>(gameInfoSize.size()))
    {
      in.bin >> gameInfo;
      const int time = gameInfo.secsRemaining;
      if(time >= 0 && time < static_cast<int>(gcTimeIndex.size()) && gcTimeIndex[time] == -1)
      {
        gcTimeIndex[time] = frame;
      }
    }
    else if(id == idProcessFinished)
      ++frame;
  }
}

void LogPlayer::countFrames()
{
  numberOfFrames = 0;
  for(int i = 0; i < getNumberOfMessages(); ++i)
  {
    queue.setSelectedMessageForReading(i);

    if(queue.getMessageID() == idProcessFinished)
    {
      ++numberOfFrames;
      numberOfMessagesWithinCompleteFrames = i + 1;
    }
  }
}

void LogPlayer::replayTypeInfo()
{
  if(typeInfo && !typeInfoReplayed)
  {
    targetQueue.out.bin << *typeInfo;
    targetQueue.out.finishMessage(idTypeInfo);
    typeInfoReplayed = true;
  }
}

void LogPlayer::merge()
{
  if(logfileMerged || logfilePath.empty())
    return;

  //parse filename and find the corresponding other logfile
  std::string file = logfilePath.substr(logfilePath.find_last_of('/') + 1);
  std::size_t prefixPos = file.find_first_of('_');
  if(prefixPos == std::string::npos)
    return; // File name does not follow pattern

  std::string processName = file.substr(0, prefixPos);
  if(processName != "Cognition" && processName != "Motion")
    return; // File name does not follow pattern

  std::string remainderName = file.substr(prefixPos);
  std::string logOtherFile = logfilePath.substr(0, logfilePath.find_last_of('/') + 1) + (processName == "Motion" ? "Cognition" : "Motion") + remainderName;

  stop();

  //copy the currently loaded logfile
  LogPlayer logCopy((MessageQueue&)*this);
  logCopy.setSize(queue.getSize());
  moveAllMessages(logCopy);
  logCopy.countFrames();
  logCopy.createIndices();

  //open other logfile
  LogPlayer logOther((MessageQueue&)*this);
  logOther.setSize(queue.getSize());
  if(!logOther.open(logOtherFile))
  {
    OUTPUT_ERROR("Could not open " << logOtherFile);
    return;
  }

  if(static_cast<unsigned long long>(logCopy.queue.usedSize) + static_cast<unsigned long long>(logOther.queue.usedSize) > std::numeric_limits<unsigned>::max())
    OUTPUT_WARNING("Merged log would exceed 4GB");

  //merge stream specification (for compatibility with old log files)
  if(typeInfo && logOther.typeInfo)
  {
    typeInfo->primitives.insert(logOther.typeInfo->primitives.begin(), logOther.typeInfo->primitives.end());
    typeInfo->enums.insert(logOther.typeInfo->enums.begin(), logOther.typeInfo->enums.end());
    typeInfo->classes.insert(logOther.typeInfo->classes.begin(), logOther.typeInfo->classes.end());
    typeInfoReplayed = false;
  }

  //TODO: ugly as hell -> use processidentifier stuff
  LogPlayer& motionLog = processName == "Motion" ? logCopy : logOther;
  LogPlayer& cognitionLog = processName == "Motion" ? logOther : logCopy;

  //Intermediate representation of messages
  struct FrameData
  {
    unsigned int time;
    int messageStart;
    int messageEnd;
  };

  std::vector<FrameData> cognitionData;
  std::vector<FrameData> motionData;
  motionData.reserve(motionLog.getNumberOfMessages());
  cognitionData.reserve(cognitionLog.getNumberOfMessages());

  FrameInfo frameInfo;
  JointAngles jointAngles;
  FrameData frameData;
  for(int i = 0; i < 2; ++i)
  {
    bool isMotionLog = i == 0;
    LogPlayer& lp = isMotionLog ? motionLog : cognitionLog;
    std::vector<FrameData>& fd = isMotionLog ? motionData : cognitionData;
    for(lp.currentMessageNumber = 0; lp.currentMessageNumber < lp.getNumberOfMessages(); ++lp.currentMessageNumber)
    {
      lp.queue.setSelectedMessageForReading(lp.currentMessageNumber);
      lp.in.text.reset();
      switch(lp.queue.getMessageID())
      {
        case idProcessBegin:
          frameData.messageStart = lp.currentMessageNumber;
          break;
        case idJointAngles:
          lp.in.bin >> jointAngles;
          frameData.time = jointAngles.timestamp;
          break;
        case idFrameInfo:
          if(isMotionLog)
            break;
          lp.in.bin >> frameInfo;
          frameData.time = frameInfo.time;
          break;
        case idProcessFinished:
          frameData.messageEnd = lp.currentMessageNumber;
          fd.push_back(frameData);
          break;
        default:
          break;
      }
    }
  }

  auto motionItr = motionData.begin();
  const auto& motionEnd = motionData.end();

  auto cognitionItr = cognitionData.begin();
  const auto& cognitionEnd = cognitionData.end();

  while(true)
  {
    if(motionItr == motionEnd)
    {
      while(cognitionItr != cognitionEnd)
      {
        for(int i = cognitionItr->messageStart; i <= cognitionItr->messageEnd; ++i)
          cognitionLog.copyMessage(i, *this);
        ++cognitionItr;
      }
      break;
    }
    if(cognitionItr == cognitionEnd)
    {
      while(motionItr != motionEnd)
      {
        for(int i = motionItr->messageStart; i <= motionItr->messageEnd; ++i)
          motionLog.copyMessage(i, *this);
        ++motionItr;
      }
      break;
    }

    if(motionItr->time < cognitionItr->time)
    {
      for(int i = motionItr->messageStart; i <= motionItr->messageEnd; ++i)
        motionLog.copyMessage(i, *this);
      ++motionItr;
    }
    else
    {
      for(int i = cognitionItr->messageStart; i <= cognitionItr->messageEnd; ++i)
        cognitionLog.copyMessage(i, *this);
      ++cognitionItr;
    }
  }
  countFrames();
  createIndices();
  logfileMerged = true;
}

void LogPlayer::loadLabels()
{
  std::string file = logfilePath.substr(logfilePath.find_last_of('/') + 1);
  std::size_t prefixPos = file.find_first_of('_');
  std::string processName = file.substr(0, prefixPos);
  if(processName == "Cognition")
  {
    ImageSet imageSet;

    std::size_t found = logfilePath.find_last_of("/\\");
    std::string logPath =  logfilePath.substr(0, found);

    DIR* dir = opendir(logPath.c_str());
    ASSERT(dir);
    struct dirent* file = readdir(dir);

    while(file != nullptr)
    {
      std::string name = file->d_name;
      if(name.rfind(".lbd") == name.size() - 4)
      {
        InMapFile stream((logfilePath.substr(0, found) + '/' + name).c_str());
        ASSERT(stream.exists());

        stream >> imageSet;
        break;
      }
      file = readdir(dir);
    }
    closedir(dir);

    if(imageSet.labelImages.size() > 0)
    {
      LogPlayer cognitionLog((MessageQueue&)*this);
      cognitionLog.setSize(queue.getSize());

      moveAllMessages(cognitionLog);
      cognitionLog.countFrames();
      cognitionLog.createIndices();

      struct FrameData
      {
        unsigned int time;
        int messageStart;
        int messageEnd;
      };

      std::vector<FrameData> cognitionData;
      cognitionData.reserve(cognitionLog.getNumberOfMessages());

      FrameInfo frameInfo;
      JointAngles jointAngles;
      FrameData frameData;

      for(cognitionLog.currentMessageNumber = 0; cognitionLog.currentMessageNumber < cognitionLog.getNumberOfMessages(); ++cognitionLog.currentMessageNumber)
      {
        cognitionLog.queue.setSelectedMessageForReading(cognitionLog.currentMessageNumber);
        cognitionLog.in.text.reset();
        switch(cognitionLog.queue.getMessageID())
        {
          case idProcessBegin:
            frameData.messageStart = cognitionLog.currentMessageNumber;
            break;
          case idFrameInfo:
            cognitionLog.in.bin >> frameInfo;
            frameData.time = frameInfo.time;
            break;
          case idProcessFinished:
            frameData.messageEnd = cognitionLog.currentMessageNumber;
            cognitionData.push_back(frameData);
            break;
          default:
            break;
        }
      }

      auto cognitionItr = cognitionData.begin();
      const auto& cognitionEnd = cognitionData.end();

      int currentIndexLogData = 0;
      while(true)
      {
        if(currentIndexLogData == static_cast<int>(imageSet.labelImages.size()))
        {
          imageSet.labelImages.at(currentIndexLogData - 1).valid = false;
          while(cognitionItr != cognitionEnd)
          {
            int i = cognitionItr->messageStart;
            while(i < cognitionItr->messageEnd)
            {
              cognitionLog.copyMessage(i++, *this);
            }
            out.bin << imageSet.labelImages.at(currentIndexLogData - 1);
            out.finishMessage(idLabelImage);
            cognitionLog.copyMessage(i, *this);
            ++cognitionItr;
          }
          break;
        }
        else
        {
          int i = cognitionItr->messageStart;
          while(i < cognitionItr->messageEnd)
          {
            cognitionLog.copyMessage(i++, *this);
          }

          if(cognitionItr->time <= imageSet.labelImages.at(currentIndexLogData).frameTime && ((cognitionItr + 1) == cognitionEnd || (cognitionItr + 1)->time > imageSet.labelImages.at(currentIndexLogData).frameTime))
          {
            imageSet.labelImages.at(currentIndexLogData).valid = true;
            out.bin << imageSet.labelImages.at(currentIndexLogData++);
            out.finishMessage(idLabelImage);
          }
          else
          {
            int index = currentIndexLogData == 0 ? currentIndexLogData : currentIndexLogData - 1;
            imageSet.labelImages.at(index).valid = false;
            out.bin << imageSet.labelImages.at(index);
            out.finishMessage(idLabelImage);
          }
          cognitionLog.copyMessage(i, *this);
          ++cognitionItr;
        }
      }
      countFrames();
      createIndices();
    }
  }
}
