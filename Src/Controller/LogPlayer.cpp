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
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/LowFrameRateImage.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Platform/SystemCall.h"
#include "Tools/Logging/LogFileFormat.h"

#include <snappy-c.h>
#include <vector>
#include <map>

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
  if(streamHandler)
  {
    delete streamHandler;
    streamHandler = nullptr;
  }
  streamSpecificationReplayed = false;
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

    if(magicByte == Logging::logFileStreamSpecification)
    {
      streamHandler = new StreamHandler;
      file >> *streamHandler;
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
        return false; //unknown magic byte
    }

    stop();
    countFrames();
    createIndices();
    logfileLoaded = true;
    return true;
  }
  logfileLoaded = false;
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
    replayStreamSpecification();

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
    currentMessageNumber = currentFrameNumber >= 0 ? frameIndex[currentFrameNumber] : 0;
    queue.setSelectedMessageForReading(currentMessageNumber);
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

bool LogPlayer::save(const std::string& fileName, const StreamHandler* streamHandler)
{
  if(state == recording)
    recordStop();

  if(!getNumberOfMessages())
    return false;

  OutBinaryFile file(fileName);
  if(file.exists())
  {
    file << Logging::logFileMessageIDs; // write magic byte to indicate message id table
    writeMessageIDs(file);
    if(streamHandler || this->streamHandler)
    {
      file << Logging::logFileStreamSpecification;
      file << (this->streamHandler ? *this->streamHandler : *streamHandler);
    }
    file << Logging::logFileUncompressed; // write magic byte to indicate uncompressed log file
    file << *this;
    return true;
  }
  return false;
}

bool LogPlayer::saveImages(const bool raw, const std::string& fileName)
{
  int i = 0;
  Image image;
  for(currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); currentMessageNumber++)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    if(queue.getMessageID() == idImage)
      in.bin >> image;
    else if(queue.getMessageID() == idJPEGImage)
    {
      JPEGImage jpegImage;
      in.bin >> jpegImage;
      jpegImage.toImage(image);
    }
    else if(queue.getMessageID() == idLowFrameRateImage)
    {
      LowFrameRateImage lowFrameRateImage;
      in.bin >> lowFrameRateImage;
      if(lowFrameRateImage.imageUpdated)
        image = lowFrameRateImage.image;
      else
        continue;
    }
    else
      continue;

    if(!saveImage(image, fileName, i++, !raw))
    {
      stop();
      return false;
    }
  }
  stop();
  return true;
}

bool LogPlayer::saveBallSpotImages(const std::string& fileName)
{
  int i = 0;

  LowFrameRateImage lfrImage;
  Image singleBallSpotImage;
  BallSpots ballspots;
  CameraInfo camerainfo;

  for(currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); currentMessageNumber++)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    switch(queue.getMessageID())
    {
      case idLowFrameRateImage:
        in.bin >> lfrImage;
        continue;
      case idBallSpots:
        in.bin >> ballspots;
        continue;
      case idCameraInfo:
        in.bin >> camerainfo;
        continue;
      case idProcessFinished: // a frame has passed and we should've both representations loaded
      {
        /* BEGIN FRAME */
        if(!lfrImage.imageUpdated) // image hasn't updated yet, so the found ballspots are useless
          continue;
        if(ballspots.ballSpots.empty()) // dump the frame when there aren't any ballspots
          continue;

        const std::string camera = camerainfo.camera == CameraInfo::Camera::upper ? "_upper" : "_lower";
        const int neuralNetImageDiameter = 16; //Depends on the value from NeuralBallPerceptor
        int oX, oY, px, py, subOriginX, subOriginY, scanRegionDimension, counter;
        unsigned char max, min;
        Image singleBallSpotImage;
        singleBallSpotImage.setResolution(neuralNetImageDiameter, neuralNetImageDiameter);
        for(auto& ballspot : ballspots.ballSpots)
        {
          oX = ballspot.x();
          oY = ballspot.y();
          scanRegionDimension = camerainfo.camera == CameraInfo::Camera::lower
                                ? (int)((48 * oY) / 185) + neuralNetImageDiameter * 3
                                : (int)((0.259 * oY) - 3);
          counter = 1;
          max = 0;
          min = 255;

          if(camerainfo.camera == CameraInfo::Camera::lower || scanRegionDimension > neuralNetImageDiameter)
          {
            subOriginX = oX - scanRegionDimension / 2;
            subOriginY = oY - scanRegionDimension / 2;

            for(int y = 0; y < neuralNetImageDiameter; ++y)
            {
              py = y * (scanRegionDimension / neuralNetImageDiameter) + subOriginY;
              for(int x = 0; x < neuralNetImageDiameter; ++x)
              {
                px = x * (scanRegionDimension / neuralNetImageDiameter) + subOriginX;
                if((px < 0 || px >= lfrImage.image.width) || (py < 0 || py >= lfrImage.image.height))
                {
                  singleBallSpotImage[y][x].y = 128;
                  if(128 > max)
                    max = 128;
                  if(128 < min)
                    min = 128;
                }
                else
                {
                  singleBallSpotImage[y][x].y = lfrImage.image[py][px].y;
                  if(lfrImage.image[py][px].y > max)
                    max = lfrImage.image[py][px].y;
                  if(lfrImage.image[py][px].y < min)
                    min = lfrImage.image[py][px].y;
                }
                counter++;
              }
            }
          }
          else
          {
            const int x1 = oX - neuralNetImageDiameter / 2;
            const int y1 = oY - neuralNetImageDiameter / 2;

            for(int y = 0; y < neuralNetImageDiameter; ++y)
            {
              py = y1 + y;
              for(int x = 0; x < neuralNetImageDiameter; ++x)
              {
                px = x1 + x;
                if((px < 0 || px >= lfrImage.image.width) || (py < 0 || py >= lfrImage.image.height))
                {
                  singleBallSpotImage[y][x].y = 128;
                  if(128 > max)
                    max = 128;
                  if(128 < min)
                    min = 128;
                }
                else
                {
                  singleBallSpotImage[y][x].y = lfrImage.image[py][px].y;
                  if(lfrImage.image[py][px].y > max)
                    max = lfrImage.image[py][px].y;
                  if(lfrImage.image[py][px].y < min)
                    min = lfrImage.image[py][px].y;
                }
                counter++;
              }
            }
          }

          unsigned char elem;
          for(int y = 0; y < neuralNetImageDiameter; ++y)
          {
            for(int x = 0; x < neuralNetImageDiameter; ++x)
            {
              elem = (singleBallSpotImage[y][x].y - min) / (max - min);
              singleBallSpotImage[y][x].y = elem <= 128 ? 0 : elem;
            }
          }
          if(!saveImage(singleBallSpotImage, fileName + camera, i++, false))
          {
            stop();
            return false;
          }
        }
        /* END FRAME */
      }
      default:
        continue;
    }
  }

  stop();
  return true;
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
      replayStreamSpecification();

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
        if(loop)  //restart in loop mode
        {
          gotoFrame(0);
          play();
        }
        else
          stop();
      }
      return true;
    }
    else
    {
      if(loop)  //restart in loop mode
      {
        gotoFrame(0);
        play();
      }
      else
        stop();
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
    temp.in.config.reset();
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
  int frameStart = -1;
  bool keepFrame = false;
  for(temp.currentMessageNumber = 0; temp.currentMessageNumber < temp.getNumberOfMessages(); ++temp.currentMessageNumber)
  {
    temp.queue.setSelectedMessageForReading(temp.currentMessageNumber);
    temp.in.config.reset();
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
  OutBinarySize gameInfoSize;
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
    else if(id == idGameInfo && queue.getMessageSize() == static_cast<int>(gameInfoSize.getSize()))
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


std::string LogPlayer::expandImageFileName(const std::string& fileName, int imageNumber)
{
  std::string name = fileName;
  std::string ext = ".png";
  std::string::size_type p = name.rfind('.');
  size_t lastDirectorySeparatorIndex = (int) name.find_last_of("\\/");
  if((int) p > (int) lastDirectorySeparatorIndex)
  {
    ext = name.substr(p);
    name = name.substr(0, p);
  }

  if(imageNumber >= 0)
  {
    char num[12];
    sprintf(num, "%03d", imageNumber);
    if(name != "" && lastDirectorySeparatorIndex != fileName.length() - 1)  // only if nonempty base name supplied
      name += "_";
    name += num + ext;
  }
  else
    name += ext;

  for(unsigned i = 0; i < name.size(); ++i)
    if(name[i] == '\\')
      name[i] = '/';
  if(name[0] != '/' && (name.size() < 2 || name[1] != ':'))
    name = File::getBHDir() + std::string("/Config/") + name;
  return name;
}

bool LogPlayer::saveImage(const Image& image, const std::string& fileName, int imageNumber, bool YUV2RGB)
{
  std::string name = expandImageFileName(fileName, imageNumber);
  const Image* r = &image;
  if(YUV2RGB)
  {
    r = new Image;
    const_cast<Image*>(r)->convertFromYCbCrToRGB(image);
  }
  QImage img(image.width, image.height, QImage::Format_RGB888);
  for(int y = 0; y < image.height; ++y)
  {
    const Image::Pixel* pSrc = &(*r)[y][0];
    unsigned char* p = img.scanLine(y);
    unsigned char* pEnd = p + 3 * image.width;

    if(YUV2RGB)
      while(p != pEnd)
      {
        *p++ = pSrc->r;
        *p++ = pSrc->g;
        *p++ = pSrc->b;
        ++pSrc;
      }
    else
      while(p != pEnd)
      {
        *p++ = pSrc->y;
        *p++ = pSrc->cb;
        *p++ = pSrc->cr;
        ++pSrc;
      }
  }
  if(YUV2RGB)
    delete r;
  return img.save(name.c_str());
}

bool LogPlayer::writeTimingData(const std::string& fileName)
{
  stop();

  std::map<unsigned short, std::string> names;/**<contains a mapping from watch id to watch name */
  std::map<unsigned, std::map<unsigned short, unsigned>> timings;/**<Contains a map from watch id to timing for each existing frame*/
  std::map<unsigned, unsigned> processStartTimes;/**< After parsing this contains the start time of each frame (frames may be missing) */
  for(int currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); currentMessageNumber++)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    if(queue.getMessageID() == idStopwatch)
    {
      //NOTE: this parser is a slightly modified version of the on in TimeInfo
      //first get the names
      unsigned short nameCount;
      in.bin >> nameCount;

      for(int i = 0; i < nameCount; ++i)
      {
        std::string watchName;
        unsigned short watchId;
        in.bin >> watchId;
        in.bin >> watchName;
        if(names.find(watchId) == names.end()) //new name
          names[watchId] = watchName;
      }

      //now get timing data
      unsigned short dataCount;
      in.bin >> dataCount;

      std::map<unsigned short, unsigned> frameTiming;
      for(int i = 0; i < dataCount; ++i)
      {
        unsigned short watchId;
        unsigned time;
        in.bin >> watchId;
        in.bin >> time;

        frameTiming[watchId] = time;
      }
      unsigned processStartTime;
      in.bin >> processStartTime;
      unsigned frameNo;
      in.bin >> frameNo;

      timings[frameNo] = frameTiming;
      processStartTimes[frameNo] = processStartTime;
    }
  }

  //now write the data to disk
  OutTextRawFile file(fileName);
  if(!file.exists())
    return false;

  std::unordered_map<unsigned short, int> columns; /**< mapping from stopwatch id to table column*/
  const std::string sep(",");
  //write header
  file << "Frame" << sep << "StartTime" << sep;
  int column = 0;
  for(auto it = names.begin(); it != names.end(); ++it, ++column)
  {
    columns[it->first] = column;
    file << it->second << sep;
  }
  file << endl;

  //write data
  for(auto it = processStartTimes.begin(); it != processStartTimes.end(); ++it)
  {
    const unsigned frameNo = it->first;
    std::vector<long> row;
    row.resize(column, -42); //-42 is a value that can never happen because all timings are unsigned
    std::map<unsigned short, unsigned>& timing = timings[frameNo];
    for(const auto& t : timing)
    {
      int colIndex = columns[t.first];
      row[colIndex] = t.second;
    }

    file << frameNo << sep << it->second << sep;
    for(long value : row)
    {
      //write timing data
      if(value == -42)
        file << "NO DATA" << sep;
      else
        file << value / 1000.0f << sep; // division by 1000 to convert to ms
    }
    file << endl;
  }
  return true;
}

bool LogPlayer::saveAudioFile(const std::string& fileName)
{
  OutBinaryFile stream(fileName);
  if(!stream.exists())
    return false;

  int frames = 0;
  AudioData audioData;
  for(currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); ++currentMessageNumber)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    if(queue.getMessageID() == idAudioData)
    {
      in.bin >> audioData;
      frames += unsigned(audioData.samples.size()) / audioData.channels;
    }
  }

  struct WAVHeader
  {
    char chunkId[4];
    int chunkSize;
    char format[4];
    char subchunk1Id[4];
    int subchunk1Size;
    short audioFormat;
    short numChannels;
    int sampleRate;
    int byteRate;
    short blockAlign;
    short bitsPerSample;
    char subchunk2Id[4];
    int subchunk2Size;
  };

  int length = sizeof(WAVHeader) + sizeof(short) * frames * audioData.channels;
  WAVHeader* header = (WAVHeader*) new char[length];
  *(unsigned*)header->chunkId = *(const unsigned*) "RIFF";
  header->chunkSize = length - 8;
  *(unsigned*)header->format = *(const unsigned*) "WAVE";

  *(unsigned*)header->subchunk1Id = *(const unsigned*) "fmt ";
  header->subchunk1Size = 16;
  header->audioFormat = 1;
  header->numChannels = (short)audioData.channels;
  header->sampleRate = audioData.sampleRate;
  header->byteRate = audioData.sampleRate * audioData.channels * sizeof(short);
  header->blockAlign = short(audioData.channels * sizeof(short));
  header->bitsPerSample = 16;

  *(unsigned*)header->subchunk2Id = *(const unsigned*) "data";
  header->subchunk2Size = frames * audioData.channels * sizeof(short);

  char* p = (char*)(header + 1);
  for(currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); ++currentMessageNumber)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    if(queue.getMessageID() == idAudioData)
    {
      in.bin >> audioData;
      memcpy(p, audioData.samples.data(), audioData.samples.size() * sizeof(short));
      p += audioData.samples.size() * sizeof(short);
    }
  }

  stream.write(header, length);
  delete[] header;

  stop();

  return true;
}

void LogPlayer::replayStreamSpecification()
{
  if(streamHandler && !streamSpecificationReplayed)
  {
    targetQueue.out.bin << *streamHandler;
    targetQueue.out.finishMessage(idStreamSpecification);
    streamSpecificationReplayed = true;
  }
}
