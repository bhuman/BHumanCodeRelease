/**
 * @file Platform/Linux/Wave.cpp
 * @author Colin Graf
 * @author Lukas Post
 * @author Thomas Röfer
 * @author Jan Blumenkamp
 * @author Lukas Plecher
 */
#include "Wave.h"

#include "Tools/Debugging/Debugging.h"

#include <string>
#include <cstring>

#if false
#include <iostream>
#define PRINT_ERROR(expr) std::cerr << "Error: " << expr << std::endl;
#else
#define PRINT_ERROR(expr)
#endif

Wave::Wave(File& file) : channels(1), sampleRate(0), numFrames(0)
{
  if(file.exists())
  {
    const size_t size = file.getSize();

    WaveFile waveFile(size);
    file.read(waveFile.buffer.data(), size);

    if(!parseWaveFile(waveFile))
    {
      PRINT_ERROR("Could not parse wave file");
      return;
    }

    sampleRate = waveFile.format->sampleRate;
    auto* sampleData = reinterpret_cast<short*>(&waveFile.data->chunkSize + 1);

    auto numSamples = waveFile.data->chunkSize / sizeof(short);
    numFrames = numSamples / std::max(static_cast<unsigned short>(1), waveFile.format->numChannels);
    channels  = 2;
    if(waveFile.format->numChannels == 1)
    {
      data.resize(numSamples * 2);
      for(short* pSrc = sampleData, *pEnd = pSrc + numSamples, *pDst = data.data(); pSrc < pEnd; ++pSrc)
      {
        *pDst++ = *pSrc;
        *pDst++ = *pSrc;
      }
    }
    else
    {
      data.reserve(numSamples);
      data = std::vector<short>(sampleData, sampleData + numSamples);
    }
  }
}

bool Wave::parseWaveFile(WaveFile& waveFile)
{
  const char* waveFileId = "WAVE";

  waveFile.master = reinterpret_cast<RiffChunk*>(waveFile.buffer.data());

  if(0 != memcmp(waveFileId, waveFile.master->waveId, 4))
  {
    return -1;
  }

  char* ptr = waveFile.buffer.data();
  for(size_t i = 12; i < (waveFile.buffer.size() - 8);)   // The first 12 bytes are occupied by the RiffChunk
  {
    size_t remainingBytes = waveFile.buffer.size() - i;

    std::string chunkType(ptr + i, ptr + i + 4);

    if(chunkType == "fmt " && remainingBytes > sizeof(FmtChunk))
      waveFile.format = reinterpret_cast<FmtChunk*>(ptr + i);
    else if(chunkType == "data" && remainingBytes > sizeof(DataChunk))
      waveFile.data = reinterpret_cast<DataChunk*>(ptr + i);

    auto* chunkSize = reinterpret_cast<uint32_t*>(ptr + i + 4);
    size_t skip = 4 + 4 + *chunkSize;
    i += skip;
  }

  if(!waveFile.format || !waveFile.data)
    return false;

  unsigned long remainingDataBytes = waveFile.buffer.size() - (reinterpret_cast<char*>(&waveFile.data->chunkSize) - waveFile.buffer.data());
  if(waveFile.data->chunkSize > remainingDataBytes)
  {
    PRINT_ERROR("Wave file data chunk claims it is " << waveFile.data->chunkSize << " bytes big, but there are only " << remainingDataBytes << " bytes remaining.");
    return false;
  }

  return true;
}

Wave::Wave(const cst_wave* wave)
{
  channels = static_cast<short>(wave->num_channels);
  sampleRate = static_cast<unsigned>(wave->sample_rate);
  numFrames = static_cast<unsigned>(wave->num_samples) / std::max(static_cast<short>(1), channels);

  if(channels == 1)
  {
    data.resize(static_cast<unsigned>(wave->num_samples * 2));
    for(short* pSrc = wave->samples, *pEnd = pSrc + wave->num_samples, *pDst = data.data(); pSrc < pEnd; ++pSrc)
    {
      *pDst++ = *pSrc;
      *pDst++ = *pSrc;
    }
    channels = 2;
  }
  else
  {
    data = std::vector<short>(wave->samples, wave->samples + wave->num_samples);
  }
}
