/**
 * @file  Platform/Linux/SoundPlayer.cpp
 * Implementation of class SoundPlayer.
 * @attention this is the Linux implementation
 * @author Colin Graf
 * @author Lukas Post
 * @author Thomas Röfer
 * @author Jan Blumenkamp
 * @author Lukas Plecher
 */

#include "SoundPlayer.h"
#include "Platform/File.h"
#include "Platform/BHAssert.h"

#include <unistd.h>

#include <string>
#include <cstring>

#if false
#include <iostream>
#define PRINT_ERROR(expr) std::cerr << "Error: " << expr << std::endl;
#else
#define PRINT_ERROR(expr)
#endif

extern "C" cst_voice* register_cmu_us_slt(const char*);

SoundPlayer SoundPlayer::soundPlayer;

Wave::Wave(File& file)
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

SoundPlayer::SoundPlayer() :
  started(false), closing(false)
{
  flite_init();
  voice = register_cmu_us_slt(nullptr);

  filePrefix = File::getBHDir();
  filePrefix += "/Config/Sounds/";
}

SoundPlayer::~SoundPlayer()
{
  if(started)
  {
    closing = true;
    sem.post();
    stop();
  }
}

void SoundPlayer::start()
{
  Thread::start(this, &SoundPlayer::main);
}

void SoundPlayer::main()
{
  Thread::nameCurrentThread("SoundPlayer");
  BH_TRACE_INIT("SoundPlayer");
  unsigned i;
  for(i = 0; i < retries; ++i)
  {
    if(snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0) >= 0)
      break;
    Thread::sleep(retryDelay);
  }
  ASSERT(i < retries);
  snd_pcm_hw_params_t* params;
  VERIFY(!snd_pcm_hw_params_malloc(&params));
  VERIFY(snd_pcm_hw_params_any(handle, params) >= 0);
  VERIFY(!snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED));
  VERIFY(!snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE));
  VERIFY(!snd_pcm_hw_params_set_rate_near(handle, params, &sampleRate, nullptr));
  VERIFY(!snd_pcm_hw_params_set_channels(handle, params, 2));
  VERIFY(!snd_pcm_hw_params(handle, params));
  VERIFY(!snd_pcm_hw_params_get_period_size(params, &periodSize, nullptr));
  snd_pcm_hw_params_free(params);

  while(isRunning() && !closing)
  {
    flush();
    VERIFY(sem.wait());
  }
  VERIFY(!snd_pcm_close(handle));
}

void SoundPlayer::playWave(const Wave& wave)
{
  auto frames = static_cast<snd_pcm_uframes_t>(wave.numFrames);
  auto* p = reinterpret_cast<const unsigned*>(wave.data.data());

  VERIFY(!snd_pcm_prepare(soundPlayer.handle));
  while(frames > 0)
  {
    snd_pcm_uframes_t periodFrames = std::min(frames, soundPlayer.periodSize);
    int err = static_cast<int>(snd_pcm_writei(soundPlayer.handle, p, periodFrames));
    if(err < 0)
    {
      err = snd_pcm_recover(soundPlayer.handle, err, 0);
      snd_strerror(err);
    }
    p += periodFrames;
    frames -= periodFrames;
  }
  VERIFY(!snd_pcm_drain(soundPlayer.handle));
}

void SoundPlayer::flush()
{
  for(;;)
  {
    SoundRequest first;
    {
      SYNC;
      if(queue.empty())
        break;
      first = queue.front();
      queue.pop_front();
    }

    playing = true;
    if(first.isTextToSpeech && !first.fileOrText.empty())
    {
      bool isStretched = first.ttsDurationStretchFactor != 1.f;
      const std::string& text = first.fileOrText;
      flite_feat_set_float(soundPlayer.voice->features, "duration_stretch", first.ttsDurationStretchFactor);

      if(isStretched)
      {
        cst_wave* rawWave = flite_text_to_wave(text.c_str(), soundPlayer.voice);
        cst_wave_rescale(rawWave, static_cast<int>(textToSpeechVolumeFactor * 65536));
        soundPlayer.playWave(Wave(rawWave));
        delete_wave(rawWave);
      }
      else
      {
        auto soundInMap = soundPlayer.synthesizedSounds.find(text);
        if(soundInMap == soundPlayer.synthesizedSounds.end())
        {
          // Sound not yet synthesized => Synthesize and insert into map
          cst_wave* rawWave = flite_text_to_wave(text.c_str(), soundPlayer.voice);
          cst_wave_rescale(rawWave, static_cast<int>(textToSpeechVolumeFactor * 65536));
          soundInMap = soundPlayer.synthesizedSounds.emplace(std::make_pair(text, Wave(rawWave))).first;
          delete_wave(rawWave);
        }
        soundPlayer.playWave(soundInMap->second);
      }
    }
    else
    {
      File file(soundPlayer.filePrefix + first.fileOrText, "rb");
      if(file.exists())
      {
        Wave wave(file);
        soundPlayer.playWave(wave);
      }
    }
    playing = false;
  }
}

int SoundPlayer::enqueue(const SoundRequest& soundRequest)
{
  int queuelen;

  {
    SYNC_WITH(soundPlayer);
    soundPlayer.queue.push_back(soundRequest);
    queuelen = static_cast<int>(soundPlayer.queue.size());
    if(!soundPlayer.started)
    {
      soundPlayer.started = true;
      soundPlayer.start();
    }
    else
      soundPlayer.sem.post();
  }
  return queuelen;
}

int SoundPlayer::play(const std::string& name)
{
  return enqueue(SoundRequest(name));
}

int SoundPlayer::say(const std::string& text, float speed)
{
  float stretchFactor = speed > 0.f ? 1.f/speed : 0.f;
  return enqueue(SoundRequest(text, stretchFactor));
}

bool SoundPlayer::isPlaying()
{
  return soundPlayer.playing;
}
