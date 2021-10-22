/**
 * @file Platform/Linux/Wave.h
 * @author Colin Graf
 * @author Lukas Post
 * @author Thomas Röfer
 * @author Jan Blumenkamp
 * @author Lukas Plecher
 */

#pragma once

#include "Platform/File.h"

#include <flite.h>
#include <cstdint>
#include <vector>

#pragma pack(push)
#pragma pack(1)

struct RiffChunk
{
  char chunkId[4];       /**< Used to identify the file format. Contains "RIFF". */
  uint32_t chunkSize;    /**< The size of this chunk (and thus the whole RIFF file) in bytes.  */
  char waveId[4];        /**< Used to identify the file format. Contains "WAVE". */
  uint32_t numChunks;    /**< The number of chunks in this file. */
};

enum WaveFormat : uint16_t
{
  WAVE_FORMAT_PCM = 0x1
};

struct FmtChunk
{
  char chunkId[4];          /**< Contains "fmt ". */
  uint32_t chunkSize;           /**< The size of this chunk in bytes. */
  WaveFormat format;        /**< Indicates the format of the samples. Only PCM is supported. */
  uint16_t numChannels;     /**< The number of interleaved channels. */
  uint32_t sampleRate;      /**< The number of samples per second. */
  uint32_t bytesPerSecond;  /**< The average number of bytes per second. */
  uint16_t blockAlign;
  uint16_t bitsPerSample;
};

struct DataChunk
{
  char chunkId[4];
  uint32_t chunkSize;
};

#pragma pack(pop)

struct WaveFile
{
  std::vector<char> buffer;
  RiffChunk* master = nullptr;
  FmtChunk* format = nullptr;
  DataChunk* data = nullptr;

  explicit WaveFile(const size_t size) : buffer(size) {}
};

struct Wave
{
  explicit Wave(File& file);
  explicit Wave(const cst_wave*);

  short channels;
  unsigned sampleRate;
  unsigned long numFrames;
  std::vector<short> data;

private:
  bool parseWaveFile(WaveFile& waveFile);
};
