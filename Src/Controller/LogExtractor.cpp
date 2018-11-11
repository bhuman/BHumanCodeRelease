/**
 * @file Controller/LogExtractor.cpp
 *
 * Implementation of class LogExtractor
 *
 * @author <a href="mailto:jan_fie@uni-bremen.de">Jan Fiedler</a>
 */

#include "LogExtractor.h"
#include "LogPlayer.h"
#include "Platform/File.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/LowFrameRateImage.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/LabelImage.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/GetUpEngineOutputLog.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Statistics.h"
#include "Tools/Logging/LogFileFormat.h"
#include "Tools/Math/Transformation.h"
#include "Tools/NeuralNetwork/NNUtilities.h"
#include "Tools/Streams/TypeInfo.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include <QImage>
#include <QDir>
#include "Representations/Infrastructure/JPEGImage.h" // Must be last include

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

  OutBinaryFile file(fileName);
  if(file.exists())
  {
    file << Logging::logFileMessageIDs; // write magic byte to indicate message id table
    logPlayer.writeMessageIDs(file);
    if(typeInfo || logPlayer.typeInfo)
    {
      file << Logging::logFileTypeInfo;
      file << (logPlayer.typeInfo ? *logPlayer.typeInfo : *typeInfo);
    }
    file << Logging::logFileUncompressed; // write magic byte to indicate uncompressed log file
    file << logPlayer;
    logPlayer.logfilePath = File::isAbsolute(fileName.c_str()) ? fileName : std::string(File::getBHDir()) + "/Config/" + fileName;
    return true;
  }
  return false;
}

bool LogExtractor::saveAudioFile(const std::string& fileName)
{
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
  for(int currentMessageNumber = 0; currentMessageNumber < logPlayer.getNumberOfMessages(); ++currentMessageNumber)
  {
    logPlayer.queue.setSelectedMessageForReading(currentMessageNumber);
    if(logPlayer.queue.getMessageID() == idAudioData)
    {
      logPlayer.in.bin >> audioData;
      memcpy(p, audioData.samples.data(), audioData.samples.size() * sizeof(short));
      p += audioData.samples.size() * sizeof(short);
    }
  }

  stream.write(header, length);
  delete[] header;

  logPlayer.stop();

  return true;
}

bool LogExtractor::saveImages(const std::string& path, const bool raw, const bool onlyPlaying)
{
  logPlayer.stop();
  DECLARE_REPRESENTATIONS_AND_MAP(
  {,
    CameraInfo,
    FrameInfo,

    // To find valid images
    FallDownState,
    GameInfo,
    Image,
    JPEGImage,
    LowFrameRateImage,
    RobotInfo,
  });

  const std::string& folderPath = createNewFolder(path);

  // Use DECLARE_REPRESENTATIONS_AND_MAP as soon as the hack is no longer needed
  return goThroughLog(
    representations,
    [&](const char frameType)
    {
      if(onlyPlaying &&
         (theGameInfo.state != STATE_PLAYING // isStateValid
          || theRobotInfo.penalty != PENALTY_NONE // isNotPenalized
          || (theFallDownState.state != FallDownState::upright
              && theFallDownState.state != FallDownState::staggering)/*isStanding*/))
        return true;

      if(theJPEGImage.timeStamp)
      {
        theJPEGImage.toImage(theImage); // Assume that Image and JPEGImage are nor logged at the same time.
        theJPEGImage.timeStamp = 0;
      }

      if(theImage.timeStamp)
      {
        CameraImage cameraImage;
        cameraImage.setReference(theImage.width, theImage.height * 2, const_cast<Image::Pixel*>(theImage[0]), theImage.timeStamp);
        cameraImage.exportImage(folderPath + (theCameraInfo.camera == CameraInfo::upper ? "upper" : "lower"),
                                theImage.timeStamp, raw ? YUYVImage::raw : YUYVImage::rgb);
        theImage.timeStamp = 0;
      }

      if(theLowFrameRateImage.imageUpdated) // gotNewImage
      {
        theLowFrameRateImage.image.exportImage(folderPath + (theCameraInfo.camera == CameraInfo::upper ? "upper" : "lower"),
                                               theLowFrameRateImage.image.timestamp, raw ? YUYVImage::raw : YUYVImage::rgb);
        theLowFrameRateImage.imageUpdated = false;
      }

      return true;
    }
  );
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
    }
  );
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
    }
  );
}

bool LogExtractor::saveWalkingData(const std::string& path)
{
  DECLARE_REPRESENTATIONS_AND_MAP(
  {,
    FrameInfo,
    WalkGenerator,
  });

  return saveCSV(path,
    [](Out& file, const std::string& sep)
    {
      file << "frame index" << sep << "isLeftPhase" << sep << "stepDuration" << sep << "time";
    },
    representations,
    [&](Out& file, const std::string& sep)
    {
      file << theFrameInfo.time << sep
           << theWalkGenerator.isLeftPhase << sep
           << theWalkGenerator.stepDuration << sep
           << theWalkGenerator.t << sep;
    }
  );
}

bool LogExtractor::saveGetUpEngineFailData(const std::string& path)
{
  DECLARE_REPRESENTATIONS_AND_MAP(
  {,
    FrameInfo,
    GetUpEngineOutput,
  });

  return saveCSV(path,
    [](Out& file, const std::string& sep)
    {
      file << "frame index" << sep << "name" << sep << "line" << sep << "direction";
    },
    representations,
    [&](Out& file, const std::string& sep)
    {
      if(theGetUpEngineOutput.failFront || theGetUpEngineOutput.failBack)
      {
        file << theFrameInfo.time << sep
             << TypeRegistry::getEnumName(theGetUpEngineOutput.name) << sep
             << theGetUpEngineOutput.lineCounter << sep
             << (theGetUpEngineOutput.failFront ? "Front" : "Back") << endl;
      }
    },
    true
  );
}

bool LogExtractor::writeTimingData(const std::string& fileName)
{
  logPlayer.stop();

  std::map<unsigned short, std::string> names;/**<contains a mapping from watch id to watch name */
  std::map<unsigned, std::map<unsigned short, unsigned>> timings;/**<Contains a map from watch id to timing for each existing frame*/
  std::map<unsigned, unsigned> processStartTimes;/**< After parsing this contains the start time of each frame (frames may be missing) */
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
      unsigned processStartTime;
      logPlayer.in.bin >> processStartTime;
      unsigned frameNo;
      logPlayer.in.bin >> frameNo;

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

bool LogExtractor::statistics(Statistics& statistics)
{
  logPlayer.stop();

  const size_t fileNameIndex = logPlayer.logfilePath.find_last_of('/') + 1;
  const QStringList logInfo(QString(logPlayer.logfilePath.substr(fileNameIndex, logPlayer.logfilePath.find_last_of('.') - fileNameIndex).c_str()).split('_'));

  statistics.initialize();
  // SYNC Different Robots but not same Robot in same Half with more than 1 Log
  const size_t robotIndex = statistics.getRobotIndex((logInfo.at(1) + (logInfo.size() > 8 ? '_' + logInfo.at(7) : "")).toStdString());

  DECLARE_REPRESENTATIONS_AND_MAP(
  {,
    BallModel,
    BallPercept,
    FallDownState,
    FrameInfo,
    GameInfo,
    GetUpEngineOutputLog,
    JointSensorData,
    KeyStates,
    MotionRequest,
    RobotHealth,
    RobotInfo,
    RobotPose,
    TeamData,
  });

  const bool result = goThroughLog(
    representations,
    [&](const char frameType)
    {
      statistics.updateLogFrame(frameType, robotIndex, Statistic::Representations(theBallModel, theBallPercept, theFallDownState, theFrameInfo, theGameInfo, theGetUpEngineOutputLog, theJointSensorData, theKeyStates, theMotionRequest, theRobotHealth, theRobotInfo, theRobotPose, theTeamData));
      return true;
    }
  );
  statistics.viewRobot(robotIndex);
  return result;
}

bool LogExtractor::saveLabeledBallSpots(const std::string& path)
{
  DECLARE_REPRESENTATIONS_AND_MAP(
  {,
    BallSpots,
    CameraInfo,
    CameraMatrix,
    LabelImage,
    LowFrameRateImage,
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
          std::vector<Annotation> annotations = theLabelImage.getLabels(ballSpot, Vector2i(diameter, diameter));
          for(const Annotation& annotation : annotations)
          {
            if(annotation.ignore)
            {
              foundIgnoreAnnotation = true;
              break;
            }
            switch(annotation.labelType)
            {
              case Annotation::Ball:
              {
                labels.at(0) = { 1, annotation.blurred, annotation.hidden };
                ss << "_ball";
                break;
              }
              case Annotation::PenaltyMark:
              {
                labels.at(1) = { 1, annotation.blurred, annotation.hidden };
                ss << "_penaltyMark";
                break;
              }
              case Annotation::FootFront:
              {
                labels.at(2) = { 1, std::min(static_cast<int>(annotation.blurred), labels.at(2).at(1)), std::min(static_cast<int>(annotation.hidden), labels.at(2).at(2)), labels.at(2).at(3) + 1 };
                ss << "_footFront";
                break;
              }
              case Annotation::FootBack:
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
            NNUtilities::extractPatch(ballSpot, Vector2i(diameter, diameter), Vector2i(32, 32), theLowFrameRateImage.image.getGrayscaled(), dest);
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
    true
  );
}

std::string LogExtractor::createNewFolder(const std::string& path) const
{
  std::string folderPath = File::isAbsolute(path.c_str()) ? path : std::string(File::getBHDir()) + "/Config/" + path;
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
    [&writeInFile, &noEndl, &file, &sep](const char frameType)
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

bool LogExtractor::goThroughLog(const std::map<const MessageID, Streamable*>& representations, const std::function<bool(const char frameType)>& executeAction)
{
  char frameType = 'a';
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
    }
    else if(message == idProcessBegin)
      logPlayer.in.bin >> frameType;
    else if(message == idProcessFinished)
    {
      if(!executeAction(frameType))
        return false;
    }
  }
  return true;
}
