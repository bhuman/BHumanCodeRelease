/**
 * @file Controller/LogExtractor.cpp
 *
 * Implementation of class LogExtractor
 *
 * @author Jan Fiedler
 */

#include "LogExtractor.h"
#include "LogPlayer.h"
#include "Platform/File.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/LabelImage.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/ImageProcessing/PatchUtilities.h"
#include "Tools/Logging/LoggingTools.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Motion/SensorData.h"
#include "Tools/Streams/TypeInfo.h"
#include <QImage>
#include <QDir>
#include <type_traits>

/**
 * Example syntax:
 * DECLARE_REPRESENTATIONS_AND_MAP(
 * {,
 *   BallModel,
 *   BallPercept,
 *   RobotPose,
 * });
 */
#define DECLARE_REPRESENTATIONS_AND_MAP(brace, ...) \
  _DECLARE_REPRESENTATIONS_AND_MAP_I(_STREAM_TUPLE_SIZE(__VA_ARGS__), __VA_ARGS__)

#define _DECLARE_REPRESENTATIONS_AND_MAP_I(n, ...) \
  _DECLARE_REPRESENTATIONS_AND_MAP_II(n, (_DECLARE_REPRESENTATIONS_AND_MAP_DECLARE, __VA_ARGS__), (_DECLARE_REPRESENTATIONS_AND_MAP_LIST, __VA_ARGS__))
#define _DECLARE_REPRESENTATIONS_AND_MAP_II(n, declare, list) \
  _STREAM_ATTR_##n declare std::map<const MessageID, Streamable*> representations = { _STREAM_ATTR_##n list };

#define _DECLARE_REPRESENTATIONS_AND_MAP_DECLARE(type) type the##type;
// The extra comma for the last representation seems to be no problem.
#define _DECLARE_REPRESENTATIONS_AND_MAP_LIST(type) { id##type, &the##type },

LogExtractor::LogExtractor(LogPlayer& logPlayer) : logPlayer(logPlayer) {}

bool LogExtractor::save(const std::string& fileName, const TypeInfo* typeInfo)
{
  if(logPlayer.state == LogPlayer::recording)
    logPlayer.recordStop();

  if(!logPlayer.getNumberOfMessages())
    return false;

  size_t index = fileName.find_last_of("\\/");
  if(index != std::string::npos)
    createNewFolder(fileName.substr(0, index));
  
  OutBinaryFile file(fileName);
  if(file.exists())
  {
    file << LoggingTools::logFileMessageIDs; // write magic byte to indicate message id table
    logPlayer.writeMessageIDs(file);
    if(typeInfo || logPlayer.typeInfo)
    {
      file << LoggingTools::logFileTypeInfo;
      file << (logPlayer.typeInfo ? *logPlayer.typeInfo : *typeInfo);
    }
    file << LoggingTools::logFileUncompressed; // write magic byte to indicate uncompressed log file
    file << logPlayer;
    logPlayer.logfilePath = File::isAbsolute(fileName.c_str()) ? fileName : std::string(File::getBHDir()) + "/Config/" + fileName;
    return true;
  }

  return false;
}

bool LogExtractor::split(const std::string& fileName, const TypeInfo* typeInfo, const int& split)
{
  int numberOfMessagesToWrite = static_cast<int>(std::ceil(logPlayer.getNumberOfMessages() / split));
  for(int i = 0; i < split; ++i)
  {
    std::string newFileName = fileName;
    newFileName.insert(fileName.find('.'), "_part_" + std::to_string(i));
    OutBinaryFile file(newFileName);
    if(file.exists())
    {
      file << LoggingTools::logFileMessageIDs;
      logPlayer.writeMessageIDs(file);
      if(typeInfo || logPlayer.typeInfo)
      {
        file << LoggingTools::logFileTypeInfo;
        file << (logPlayer.typeInfo ? *logPlayer.typeInfo : *typeInfo);
      }

      file << LoggingTools::logFileUncompressed;
      MessageQueue newQueue;
      newQueue.setSize(logPlayer.getSize());
      for(int j = 0; j < numberOfMessagesToWrite; j++)
      {
        if(i * numberOfMessagesToWrite + j > logPlayer.getNumberOfMessages()) break;

        logPlayer.copyMessage(i * numberOfMessagesToWrite + j, newQueue);
      }
      file << newQueue;
    }
    else return false;
  }
  return true;
}

bool LogExtractor::saveAudioFile(const std::string& fileName)
{
  logPlayer.stop();

  OutBinaryFile stream(fileName);
  if(!stream.exists())
    return false;

  int frames = 0;
  AudioData audioData;
  for(int currentMessageNumber = 0; currentMessageNumber < logPlayer.getNumberOfMessages(); ++currentMessageNumber)
  {
    logPlayer.queue.setSelectedMessageForReading(currentMessageNumber);
    if(logPlayer.queue.getMessageID() == idAudioData)
    {
      logPlayer.in.bin >> audioData;
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

  int length = sizeof(WAVHeader) + sizeof(AudioData::Sample) * frames * audioData.channels;
  WAVHeader* header = reinterpret_cast<WAVHeader*>(new char[length]);
  *reinterpret_cast<unsigned*>(header->chunkId) = *reinterpret_cast<const unsigned*>("RIFF");
  header->chunkSize = length - 8;
  *reinterpret_cast<unsigned*>(header->format) = *reinterpret_cast<const unsigned*>("WAVE");

  *reinterpret_cast<unsigned*>(header->subchunk1Id) = *reinterpret_cast<const unsigned*>("fmt ");
  header->subchunk1Size = 16;
  static_assert(std::is_same<AudioData::Sample, short>::value
                || std::is_same<AudioData::Sample, float>::value, "Wrong audio sample type");
  header->audioFormat = std::is_same<AudioData::Sample, short>::value ? 1 : 3;
  header->numChannels = static_cast<short>(audioData.channels);
  header->sampleRate = audioData.sampleRate;
  header->byteRate = audioData.sampleRate * audioData.channels * sizeof(AudioData::Sample);
  header->blockAlign = short(audioData.channels * sizeof(AudioData::Sample));
  header->bitsPerSample = short(sizeof(AudioData::Sample) * 8);

  *reinterpret_cast<unsigned*>(header->subchunk2Id) = *reinterpret_cast<const unsigned*>("data");
  header->subchunk2Size = frames * audioData.channels * sizeof(AudioData::Sample);

  char* p = reinterpret_cast<char*>(header + 1);
  for(int currentMessageNumber = 0; currentMessageNumber < logPlayer.getNumberOfMessages(); ++currentMessageNumber)
  {
    logPlayer.queue.setSelectedMessageForReading(currentMessageNumber);
    if(logPlayer.queue.getMessageID() == idAudioData)
    {
      logPlayer.in.bin >> audioData;
      memcpy(p, audioData.samples.data(), audioData.samples.size() * sizeof(AudioData::Sample));
      p += audioData.samples.size() * sizeof(AudioData::Sample);
    }
  }

  stream.write(header, length);
  delete[] header;

  return true;
}

bool LogExtractor::saveImages(const std::string& path, const bool raw, const bool onlyPlaying, const int takeEachNthFrame = 1)
{
  class CRCLut : public std::array<unsigned int, 256>
  {
  public:
    CRCLut() : std::array<unsigned int, 256>()
    {
      for(unsigned int n = 0; n < 256; n++)
      {
        unsigned int c = n;
        for(unsigned int k = 0; k < 8; k++)
        {
          if(c & 1)
            c = 0xedb88320L ^ (c >> 1);
          else
            c = c >> 1;
        }
        (*this)[n] = c;
      }
    }
  };

  class CRC
  {
  private:
    unsigned int crc;
  public:
    CRC() : crc(0xffffffff) {}
    CRC(const unsigned int initialCrc) : crc(initialCrc) {}

    CRC update(const CRCLut& lut, const void* data, const size_t size) const
    {
      const unsigned char* dataPtr = reinterpret_cast<const unsigned char*>(data);
      unsigned int crc = this->crc;

      for(size_t i = 0; i < size; i++)
        crc = lut[(crc ^ dataPtr[i]) & 0xff] ^ (crc >> 8);

      return CRC(crc);
    }

    unsigned int finish() const
    {
      return crc ^ 0xffffffff;
    }
  };

  logPlayer.stop();
  DECLARE_REPRESENTATIONS_AND_MAP(
  {,
    CameraInfo,
    CameraMatrix,
    FrameInfo,
    ImageCoordinateSystem,

    // To find valid images
    FallDownState,
    GameInfo,
    CameraImage,
    JPEGImage,
    RobotInfo,
  });

  const std::string& folderPath = createNewFolder(path);

  const CRCLut crcLut;

  int skippedImageCount = 0;

  // Use DECLARE_REPRESENTATIONS_AND_MAP as soon as the hack is no longer needed
  return goThroughLog(
           representations,
           [&](const std::string& frameType)
  {
    if(onlyPlaying &&
       (theGameInfo.state != STATE_PLAYING // isStateValid
        || theRobotInfo.penalty != PENALTY_NONE // isNotPenalized
        || (theFallDownState.state != FallDownState::upright
            && theFallDownState.state != FallDownState::staggering)/*isStanding*/))
      return true;

    if(theJPEGImage.timestamp)
    {
      theJPEGImage.toCameraImage(theCameraImage); // Assume that CameraImage and JPEGImage are not logged at the same time.
      theJPEGImage.timestamp = 0;
    }

    CameraImage* imageToExport = nullptr;

    if(theCameraImage.timestamp)
      imageToExport = &theCameraImage;

    if(imageToExport)
    {
      // Frame skipping: only count frames if they are from the upper camera so
      // that always a pair of lower and upper frames is saved
      if(theCameraInfo.camera == CameraInfo::upper && ++skippedImageCount == takeEachNthFrame)
        skippedImageCount = 0;
      if(skippedImageCount != 0)
        return true;

      // Open PNG file
      const std::string filename = CameraImage::expandImageFileName(folderPath + (theCameraInfo.camera == CameraInfo::upper ? "upper" : "lower"), imageToExport->timestamp);
      QFile qfile(filename.c_str());
      qfile.open(QIODevice::WriteOnly);

      // Write image
      imageToExport->exportImage(qfile, raw ? YUYVImage::raw : YUYVImage::rgb);
      imageToExport->timestamp = 0;

      // Remove IEND chunk
      qfile.resize(qfile.size() - 12);

      // Write metadata
      OutBinaryMemory metaData;
      metaData << theCameraInfo;
      metaData << theCameraMatrix;
      metaData << theImageCoordinateSystem;
      const unsigned int size = static_cast<unsigned int>(metaData.size());
      for(size_t i = 0; i < 4; i++)
        qfile.putChar(reinterpret_cast<const char*>(&size)[3 - i]);
      qfile.write("bhMn");
      qfile.write(metaData.data(), metaData.size());
      const unsigned int crc = CRC().update(crcLut, "bhMn", 4).update(crcLut, metaData.data(), metaData.size()).finish();
      for(size_t i = 0; i < 4; i++)
        qfile.putChar(reinterpret_cast<const char*>(&crc)[3 - i]);

      // Write IEND chunk
      const std::array<char, 12> endChunk{ 0, 0, 0, 0, 'I', 'E', 'N', 'D', char(0xae), char(0x42), char(0x60), char(0x82) };
      qfile.write(endChunk.data(), endChunk.size());
      qfile.close();
    }

    return true;
  });
}

bool LogExtractor::saveInertialSensorData(const std::string& path)
{
  DECLARE_REPRESENTATIONS_AND_MAP(
  {,
    FrameInfo,
    InertialSensorData,
  });

  return saveCSV(path,
                 [](Out& file, const std::string& sep)
  {
    file << "# frame index" << sep << "gyro x" << sep << "gyro y" << sep << "gyro z" << sep << "acc x" << sep << "acc y" << sep << "acc z" << sep << "angle x" << sep << "angle y";
  },
  representations,
  [&](Out& file, const std::string& sep)
  {
    file << theFrameInfo.time << sep
         << static_cast<float>(theInertialSensorData.gyro.x()) << sep
         << static_cast<float>(theInertialSensorData.gyro.y()) << sep
         << static_cast<float>(theInertialSensorData.gyro.z()) << sep
         << theInertialSensorData.acc.x() << sep
         << theInertialSensorData.acc.y() << sep
         << theInertialSensorData.acc.z() << sep
         << static_cast<float>(theInertialSensorData.angle.x()) << sep
         << static_cast<float>(theInertialSensorData.angle.y());
  });
}

bool LogExtractor::saveJointAngleData(const std::string& path)
{
  DECLARE_REPRESENTATIONS_AND_MAP(
  {,
    FrameInfo,
    JointAngles,
    JointRequest,
  });

  return saveCSV(path,
                 [](Out& file, const std::string& sep)
  {
    file << "frame index";
    for(int i = 0; i < Joints::numOfJoints; i++)
    {
      file << sep << "request " << i << sep << "angle " << i;
    }
  },
  representations,
  [&](Out& file, const std::string& sep)
  {
    file << theFrameInfo.time;
    for(int i = 0; i < Joints::numOfJoints; i++)
    {
      file << sep << theJointRequest.angles[i].toDegrees() << sep << theJointAngles.angles[i].toDegrees();
    }
  });
}

bool LogExtractor::writeTimingData(const std::string& fileName)
{
  logPlayer.stop();

  std::map<unsigned short, std::string> names;/**<contains a mapping from watch id to watch name */
  std::map<unsigned, std::map<unsigned short, unsigned>> timings;/**<Contains a map from watch id to timing for each existing frame*/
  std::map<unsigned, unsigned> threadStartTimes;/**< After parsing this contains the start time of each frame (frames may be missing) */
  for(int currentMessageNumber = 0; currentMessageNumber < logPlayer.getNumberOfMessages(); currentMessageNumber++)
  {
    logPlayer.queue.setSelectedMessageForReading(currentMessageNumber);
    if(logPlayer.queue.getMessageID() == idStopwatch)
    {
      //NOTE: this parser is a slightly modified version of the on in TimeInfo
      //first get the names
      unsigned short nameCount;
      logPlayer.in.bin >> nameCount;

      for(unsigned short i = 0; i < nameCount; ++i)
      {
        std::string watchName;
        unsigned short watchId;
        logPlayer.in.bin >> watchId;
        logPlayer.in.bin >> watchName;
        if(names.find(watchId) == names.end()) //new name
          names[watchId] = watchName;
      }

      //now get timing data
      unsigned short dataCount;
      logPlayer.in.bin >> dataCount;

      std::map<unsigned short, unsigned> frameTiming;
      for(unsigned short i = 0; i < dataCount; ++i)
      {
        unsigned short watchId;
        unsigned time;
        logPlayer.in.bin >> watchId;
        logPlayer.in.bin >> time;

        frameTiming[watchId] = time;
      }
      unsigned threadStartTime;
      logPlayer.in.bin >> threadStartTime;
      unsigned frameNo;
      logPlayer.in.bin >> frameNo;

      timings[frameNo] = frameTiming;
      threadStartTimes[frameNo] = threadStartTime;
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
  for(auto it = threadStartTimes.begin(); it != threadStartTimes.end(); ++it)
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

bool LogExtractor::saveLabeledBallSpots(const std::string& path)
{
  DECLARE_REPRESENTATIONS_AND_MAP(
  {,
    BallSpots,
    CameraInfo,
    CameraMatrix,
    LabelImage,
    JPEGImage,
  });
  BallSpecification ballSpecification;

  int imageNumber = 0;
                    const std::string& folderPath = createNewFolder(path.substr(0, path.rfind(".")) + "/");

                    return saveCSV(path,
                                   [](Out& file, const std::string& sep)
  {
    file << "# imagenumber" << sep << "frametime" << sep << "camera" << sep << "ball" << sep << "blurred" << sep << "hidden" << sep << "penaltyMark" << sep << "blurred" << sep << "hidden" << sep << "footFront" << sep << "blurred" << sep << "hidden" << sep << "counter footFront" << sep << "footBack" << sep << "blurred" << sep << "hidden" << sep << "counter footBack" << sep << "x image" << sep << "y image" << sep << "x field" << sep << "y field" << sep << "estimated distance";
  },
  representations,
  [&](Out& file, const std::string& sep)
  {
    if(theLabelImage.valid)
    {
      for(const Vector2i& ballSpot : theBallSpots.ballSpots)
      {
        std::stringstream ss;
        GrayscaledImage dest;
        bool foundIgnoreAnnotation = false;
        std::vector< std::vector<int> > labels = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 1, 1, 0 }, { 0, 1, 1, 0 } };

        ss << folderPath << imageNumber;

        const float diameter = IISC::getImageBallRadiusByCenter(ballSpot.cast<float>(), theCameraInfo, theCameraMatrix, ballSpecification) * 3.5f;
        std::vector<LabelImage::Annotation> annotations = theLabelImage.getLabels(ballSpot, Vector2i(diameter, diameter));
        for(const auto& annotation : annotations)
        {
          if(annotation.ignore)
          {
            foundIgnoreAnnotation = true;
            break;
          }
          switch(annotation.labelType)
          {
            case LabelImage::Annotation::Ball:
            {
              labels.at(0) = { 1, annotation.blurred, annotation.hidden };
              ss << "_ball";
              break;
            }
            case LabelImage::Annotation::PenaltyMark:
            {
              labels.at(1) = { 1, annotation.blurred, annotation.hidden };
              ss << "_penaltyMark";
              break;
            }
            case LabelImage::Annotation::FootFront:
            {
              labels.at(2) = { 1, std::min(static_cast<int>(annotation.blurred), labels.at(2).at(1)), std::min(static_cast<int>(annotation.hidden), labels.at(2).at(2)), labels.at(2).at(3) + 1 };
              ss << "_footFront";
              break;
            }
            case LabelImage::Annotation::FootBack:
            {
              labels.at(3) = { 1, std::min(static_cast<int>(annotation.blurred), labels.at(3).at(1)), std::min(static_cast<int>(annotation.hidden), labels.at(3).at(2)), labels.at(3).at(3) + 1 };
              ss << "_footBack";
              break;
            }
            default:
              continue;
          }
        }
        bool foundMatchingAnnotation = labels.at(0).at(0) || labels.at(1).at(0) || labels.at(2).at(0) || labels.at(3).at(0);
        if(!foundIgnoreAnnotation || foundMatchingAnnotation)
        {
          Vector2f ballSpotOnField;
          if(!Transformation::imageToRobotHorizontalPlane(ballSpot.cast<float>(), ballSpecification.radius, theCameraMatrix, theCameraInfo, ballSpotOnField))
            continue;
          CameraImage cameraImage;
          theJPEGImage.toCameraImage(cameraImage);
          PatchUtilities::extractPatch(ballSpot, Vector2i(diameter, diameter), Vector2i(32, 32), cameraImage.getGrayscaled(), dest);
          dest.exportImage(ss.str(), theLabelImage.frameTime, GrayscaledImage::grayscale);
          file << imageNumber << sep
               << theLabelImage.frameTime << sep
               << theCameraInfo.camera << sep
               << labels.at(0).at(0) << sep << labels.at(0).at(1) << sep << labels.at(0).at(2) << sep
               << labels.at(1).at(0) << sep << labels.at(1).at(1) << sep << labels.at(1).at(2) << sep
               << labels.at(2).at(0) << sep << labels.at(2).at(1) << sep << labels.at(2).at(2) << sep << labels.at(2).at(3) << sep
               << labels.at(3).at(0) << sep << labels.at(3).at(1) << sep << labels.at(3).at(2) << sep << labels.at(3).at(3) << sep
               << ballSpot.x() << sep << ballSpot.y() << sep
               << static_cast<int>(ballSpotOnField.x()) << sep << static_cast<int>(ballSpotOnField.y()) << sep;
          Vector2f relativePoint;
          if(foundMatchingAnnotation && Transformation::imageToRobotHorizontalPlane(ballSpot.cast<float>(), 0.f, theCameraMatrix, theCameraInfo, relativePoint))
            file << relativePoint.norm();
          else
            file << -1;
          file << endl;
          imageNumber++;
        }
      }
    }
    return true;
  },
  true);
}

std::string LogExtractor::createNewFolder(const std::string& prefix) const
{
  std::string folderPath = File::isAbsolute(prefix.c_str()) ? prefix : std::string(File::getBHDir()) + "/Config/" + prefix;
  QDir().mkpath(folderPath.c_str());
  return folderPath;
}

bool LogExtractor::saveCSV(const std::string& fileName, const std::function<void(Out& file, const std::string& sep)>& writeHeader, const std::map<const MessageID, Streamable*>& representations, const std::function<void(Out& file, const std::string& sep)>& writeInFile, const bool noEndl)
{
  logPlayer.stop(); // Just reset the LogPlayer to start
  const std::string name = File::isAbsolute(fileName.c_str()) ? fileName : std::string(File::getBHDir()) + "/Config/" + fileName;
  OutTextRawFile file(name);
  if(!file.exists())
    return false;
  const std::string sep = ";";
  writeHeader(file, sep);
  file << endl;

  const bool finished = goThroughLog(
                          representations,
                          [&writeInFile, &noEndl, &file, &sep](const std::string& frameType)
  {
    writeInFile(file, sep);
    if(!noEndl)
      file << endl;
    return true;
  }
                        );
  OUTPUT_TEXT("File was created successfully: " << name);
  return finished;
}

bool LogExtractor::goThroughLog(const std::map<const MessageID, Streamable*>& representations, const std::function<bool(const std::string& frameType)>& executeAction)
{
  std::string frameType;
  bool filled = false;
  for(int currentMessageNumber = 0; currentMessageNumber < logPlayer.getNumberOfMessages(); currentMessageNumber++)
  {
    logPlayer.queue.setSelectedMessageForReading(currentMessageNumber);
    MessageID message = logPlayer.queue.getMessageID();
    auto repr = representations.find(message);
    // repr == end() if not found
    if(repr != representations.end())
    {
      // Does not convert logs automatically now, can be found in LogDataProvider
      logPlayer.in.bin >> *(repr->second);
      filled = true;
    }
    else if(message == idFrameBegin)
    {
      frameType = logPlayer.in.readThreadIdentifier();
      filled = false;
    }
    else if(message == idFrameFinished && filled)
    {
      if(!executeAction(frameType))
        return false;
    }
  }
  return true;
}
