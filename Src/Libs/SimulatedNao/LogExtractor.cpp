/**
 * @file LogExtractor.cpp
 *
 * Implementation of class LogExtractor
 *
 * @author Jan Fiedler
 */

#include "LogExtractor.h"
#include "LogPlayback/ImageExport.h"
#include "LogPlayback/Log.h"
#include "Platform/File.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Sensing/FallDownState.h"
#include <filesystem>

LogExtractor::LogExtractor(LogPlayer& logPlayer) : logPlayer(logPlayer) {}

bool LogExtractor::saveAudioFile(const std::string& fileName)
{
  OutBinaryFile stream(fileName);
  if(!stream.exists())
    return false;

  int frames = 0;
  AudioData audioData;
  for(MessageQueue::Message message : logPlayer)
    if(logPlayer.id(message) == idAudioData)
    {
      message.bin() >> audioData;
      frames += unsigned(audioData.samples.size()) / audioData.channels;
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
  for(MessageQueue::Message message : logPlayer)
    if(logPlayer.id(message) == idAudioData)
    {
      message.bin() >> audioData;
      memcpy(p, audioData.samples.data(), audioData.samples.size() * sizeof(AudioData::Sample));
      p += audioData.samples.size() * sizeof(AudioData::Sample);
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

  const Log log(logPlayer);
  std::string folderPath = File::isAbsolute(path.c_str()) ? path : std::string(File::getBHDir()) + "/Config/" + path;
  std::filesystem::create_directories(folderPath);

  const CRCLut crcLut;

  int skippedImageCount = 0;
  GameState theGameState;
  FallDownState theFallDownState;
  CameraImage theUnpackedJPEGImage;

  for(Log::Frame frame : log)
  {
    if(onlyPlaying)
    {
      if(frame.contains(idGameState))
        theGameState = frame[idGameState];
      if(frame.contains(idFallDownState))
        theFallDownState = frame[idFallDownState];
      if(!theGameState.isPlaying() // isStateValid
         || theGameState.isPenalized() // isNotPenalized
         || (theFallDownState.state != FallDownState::upright
             && theFallDownState.state != FallDownState::staggering)/*isStanding*/)
        continue;
    }

    if(frame.contains(idCameraImage) || frame.contains(idJPEGImage))
    {
      const CameraInfo& theCameraInfo = frame[idCameraInfo];

      if(theCameraInfo.camera == CameraInfo::upper && ++skippedImageCount == takeEachNthFrame)
        skippedImageCount = 0;
      if(skippedImageCount)
        continue;

      const CameraImage* imageToExport = &theUnpackedJPEGImage;
      if(frame.contains(idJPEGImage))
        frame[idJPEGImage].cast<JPEGImage>().toCameraImage(theUnpackedJPEGImage);
      else
        imageToExport = &frame[idCameraImage].cast<CameraImage>();

      // Open PNG file
      const std::string filename = ImageExport::expandImageFileName(folderPath + TypeRegistry::getEnumName(theCameraInfo.camera), imageToExport->timestamp);
      QFile qFile(filename.c_str());
      qFile.open(QIODevice::WriteOnly);

      // Write image
      ImageExport::exportImage(*imageToExport, qFile, "PNG", raw ? ImageExport::raw : ImageExport::rgb);

      // Remove IEND chunk
      qFile.resize(qFile.size() - 12);

      // Write metadata
      OutBinaryMemory metaData;
      metaData << frame[idCameraInfo].cast<CameraInfo>();
      metaData << frame[idCameraMatrix].cast<CameraMatrix>();
      metaData << frame[idImageCoordinateSystem].cast<ImageCoordinateSystem>();
      const unsigned int size = static_cast<unsigned int>(metaData.size());
      for(size_t i = 0; i < 4; i++)
        qFile.putChar(reinterpret_cast<const char*>(&size)[3 - i]);
      qFile.write("bhMn");
      qFile.write(metaData.data(), metaData.size());
      const unsigned int crc = CRC().update(crcLut, "bhMn", 4).update(crcLut, metaData.data(), metaData.size()).finish();
      for(size_t i = 0; i < 4; i++)
        qFile.putChar(reinterpret_cast<const char*>(&crc)[3 - i]);

      // Write IEND chunk
      const std::array<char, 12> endChunk{ 0, 0, 0, 0, 'I', 'E', 'N', 'D', char(0xae), char(0x42), char(0x60), char(0x82) };
      qFile.write(endChunk.data(), endChunk.size());
      qFile.close();
    }
  }
  return true;
}
