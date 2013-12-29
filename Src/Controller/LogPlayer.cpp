/**
* @file Controller/LogPlayer.cpp
*
* Implementation of class LogPlayer
*
* @author Martin Lötzsch
*/

#include <QImage>
#include "LogPlayer.h"
#include "Representations/Perception/JPEGImage.h"
#include "Platform/SystemCall.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Tools/Debugging/LogFileFormat.h"
#include "snappy-c.h"
#include <vector>
#include <list>
#include <map>
using namespace std;

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
  loop = true; //default: loop enabled
}

bool LogPlayer::open(const char* fileName)
{
  InBinaryFile file(fileName);
  unsigned long long totalUncompressedSize = 0;
  if(file.exists())
  {
    clear();

    char magicByte;
    file >> magicByte;
    
    switch(magicByte)
    {
      case logFileRegular: //regular log file
        file >> *this;
        break;
      case logFileCompressed://compressed log file
        while(!file.eof())
        {
          unsigned compressedSize;
          file >> compressedSize;
          ASSERT(compressedSize > 0);
          std::vector<char> compressedBuffer;
          compressedBuffer.resize(compressedSize);
          file.read(&compressedBuffer[0], (int)compressedSize);

          size_t uncompressedSize = 0;
          snappy_uncompressed_length(&compressedBuffer[0], compressedSize, &uncompressedSize);
          std::vector<char> uncompressBuffer;
          uncompressBuffer.resize(uncompressedSize);
          snappy_uncompress(&compressedBuffer[0], compressedSize, &uncompressBuffer[0], &uncompressedSize);
          InBinaryMemory mem(&uncompressBuffer[0], uncompressedSize);
          totalUncompressedSize += uncompressedSize;

          //if this assert is triggered the log file is too big. The maximum size is set
          //by the RobotConsole.
          ASSERT(totalUncompressedSize <= (unsigned long long) queue.getSize());
          mem >> *this;
        }
        break;
      default:
        ASSERT(FALSE); //unknown magic byte
    }
    
    stop();
    countFrames();
    createFrameIndex();
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
  if(state == paused && currentFrameNumber > 0)
  {
    currentMessageNumber = frameIndex[--currentFrameNumber];
    queue.setSelectedMessageForReading(currentMessageNumber);
    stepRepeat();
  }
}

void LogPlayer::stepImageBackward()
{
  pause();
  if(state == paused && currentFrameNumber > 0)
  {
    int lastImageFrameNumber = this->lastImageFrameNumber;
    do
      stepBackward();
    while(lastImageFrameNumber == this->lastImageFrameNumber && currentFrameNumber > 0);
  }
}

void LogPlayer::stepForward()
{
  pause();
  if(state == paused && currentFrameNumber < numberOfFrames - 1 && currentMessageNumber < numberOfMessagesWithinCompleteFrames - 1)
  {
    do
    {
      copyMessage(++currentMessageNumber, targetQueue);
      if(queue.getMessageID() == idImage || queue.getMessageID() == idJPEGImage)
        lastImageFrameNumber = currentFrameNumber;
    }
    while(queue.getMessageID() != idProcessFinished);
    ++currentFrameNumber;
  }
}

void LogPlayer::stepImageForward()
{
  pause();
  if(state == paused && currentFrameNumber < numberOfFrames - 1)
  {
    int lastImageFrameNumber = this->lastImageFrameNumber;
    do
      stepForward();
    while(lastImageFrameNumber == this->lastImageFrameNumber && currentFrameNumber < numberOfFrames - 1);
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
    currentMessageNumber = currentFrameNumber >= 0 ? frameIndex[currentFrameNumber] : 0;
    queue.setSelectedMessageForReading(currentMessageNumber);
    stepForward();
  }
}

bool LogPlayer::save(const char* fileName)
{
  if(state == recording)
    recordStop();

  if(!getNumberOfMessages())
    return false;

  OutBinaryFile file(fileName);
  if(file.exists())
  {
    file << logFileRegular; //write magic byte to indicates regular log file
    file << *this;
    return true;
  }
  return false;
}

bool LogPlayer::saveImages(const bool raw, const char* fileName)
{
  int i = 0;
  for(currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); currentMessageNumber++)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    Image image;
    if(queue.getMessageID() == idImage)
    {
      in.bin >> image;
    }
    else if(queue.getMessageID() == idJPEGImage)
    {
      JPEGImage jpegImage;
      in.bin >> jpegImage;
      jpegImage.toImage(image);
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


void LogPlayer::setLoop(bool loop_)
{
  loop = loop_;
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
      do
      {
        copyMessage(++currentMessageNumber, targetQueue);
        if(queue.getMessageID() == idImage || queue.getMessageID() == idJPEGImage)
          lastImageFrameNumber = currentFrameNumber;
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

void LogPlayer::keep(MessageID* messageIDs)
{
  stop();
  LogPlayer temp((MessageQueue&) *this);
  temp.setSize(queue.getSize());
  moveAllMessages(temp);
  for(temp.currentMessageNumber = 0; temp.currentMessageNumber < temp.getNumberOfMessages(); ++temp.currentMessageNumber)
  {
    temp.queue.setSelectedMessageForReading(temp.currentMessageNumber);
    MessageID* m = messageIDs;
    while(*m)
    {
      if(temp.queue.getMessageID() == *m ||
         temp.queue.getMessageID() == idProcessBegin ||
         temp.queue.getMessageID() == idProcessFinished)
      {
        temp.copyMessage(temp.currentMessageNumber, *this);
        break;
      }
      ++m;
    }
  }
  countFrames();
  if(!frameIndex.empty())
    createFrameIndex();
}

void LogPlayer::remove(MessageID* messageIDs)
{
  stop();
  LogPlayer temp((MessageQueue&) *this);
  temp.setSize(queue.getSize());
  moveAllMessages(temp);
  for(temp.currentMessageNumber = 0; temp.currentMessageNumber < temp.getNumberOfMessages(); ++temp.currentMessageNumber)
  {
    temp.queue.setSelectedMessageForReading(temp.currentMessageNumber);
    MessageID* m = messageIDs;
    while(*m)
    {
      if(temp.queue.getMessageID() == *m)
        break;
      ++m;
    }
    if(!*m)
      temp.copyMessage(temp.currentMessageNumber, *this);
  }
  countFrames();
  if(!frameIndex.empty())
    createFrameIndex();
}

void LogPlayer::statistics(int frequency[numOfDataMessageIDs])
{
  for(int i = 0; i < numOfDataMessageIDs; ++i)
    frequency[i] = 0;

  if(getNumberOfMessages() > 0)
  {
    int current = queue.getSelectedMessageForReading();
    for(int i = 0; i < getNumberOfMessages(); ++i)
    {
      queue.setSelectedMessageForReading(i);
      ASSERT(queue.getMessageID() < numOfDataMessageIDs);
      ++frequency[queue.getMessageID()];
    }
    queue.setSelectedMessageForReading(current);
  }
}

void LogPlayer::createFrameIndex()
{
  queue.createIndex();
  frameIndex.clear();
  frameIndex.reserve(numberOfFrames);
  for(int i = 0; i < getNumberOfMessages(); ++i)
  {
    queue.setSelectedMessageForReading(i);
    if(queue.getMessageID() == idProcessBegin)
      frameIndex.push_back(i);
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

std::string LogPlayer::expandImageFileName(const char* fileName, int imageNumber)
{
  std::string name(fileName);
  std::string ext(".bmp");
  std::string::size_type p = (int) name.rfind('.');
  if((int) p > (int) name.find_last_of("\\/"))
  {
    ext = name.substr(p);
    name = name.substr(0, p);
  }

  if(imageNumber >= 0)
  {
    char num[12];
    sprintf(num, "%03d", imageNumber);
    name = name + "_" + num + ext;
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

bool LogPlayer::saveImage(const Image& image, const char* fileName, int imageNumber, bool YUV2RGB)
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
    unsigned char* p = img.scanLine(y),
                 * pEnd = p + 3 * image.width;
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

  map<unsigned short, string> names;/**<contains a mapping from watch id to watch name */
  map<unsigned, map<unsigned short, unsigned>> timings;/**<Contains a map from watch id to timing for each existing frame*/
  map<unsigned, unsigned> processStartTimes;/**< After parsing this contains the start time of each frame (frames may be missing) */
  for(int currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); currentMessageNumber++)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    if(queue.getMessageID() == idStopwatch)
    {//NOTE: this parser is a slightly modified version of the on in TimeInfo
      //first get the names
      unsigned short nameCount;
      in.bin >> nameCount;

      for(int i = 0; i < nameCount; ++i)
      {
        string watchName;
        unsigned short watchId;
        in.bin >> watchId;
        in.bin >> watchName;
        if(names.find(watchId) == names.end()) //new name
        {
          names[watchId] = watchName;
        }
      }

      //now get timing data
      unsigned short dataCount;
      in.bin >> dataCount;

      map<unsigned short, unsigned> frameTiming;
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

  unordered_map<unsigned short, int> columns; /**< mapping from stopwatch id to table column*/
  const string sep(",");
  //write header
  file << "Frame" << sep << "StartTime" << sep;
  int column = 0;
  for(map<unsigned short, string>::iterator it = names.begin(); it != names.end(); ++it, ++column)
  {
    columns[it->first] = column;
    file << it->second << sep;
  }
  file << endl;

  //write data
  map<unsigned, unsigned>::iterator it;
  for(it = processStartTimes.begin(); it != processStartTimes.end(); ++it)
  {
    const unsigned frameNo = it->first;
    vector<long> row;
    row.resize(column, -42); //-42 is a value that can never happen because all timings are unsigned
    map<unsigned short, unsigned>& timing = timings[frameNo];
    for(auto t : timing)
    {
      int colIndex = columns[t.first];
      row[colIndex] = t.second;
    }

    file << frameNo << sep << it->second << sep;
    for(long value : row)
    {//write timing data
      if(value == -42)
      {
        file << "NO DATA" << sep;
      }
      else
      {
        file << value / 1000.0f << sep; // division by 1000 to convert to ms
      }
    }
    file << endl;
  }
  return true;
}
