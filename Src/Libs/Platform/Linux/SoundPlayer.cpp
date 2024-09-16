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

#ifdef HAVE_ALSA

#include "SoundPlayer.h"
#include "Platform/File.h"
#include "Platform/BHAssert.h"

#include <unistd.h>

#include <string>
#include <cstring>

#if false
#include "Streaming/Output.h"
#define PRINT_ERROR(expr) OUTPUT_ERROR(expr)
#else
#define PRINT_ERROR(expr)
#endif

extern "C" cst_voice* register_cmu_us_slt(const char*);

SoundPlayer SoundPlayer::soundPlayer;

WaveFile::WaveFile(File& file) :
  buffer(file.getSize())
{
  file.read(buffer.data(), buffer.size());
}

bool WaveFile::parse()
{
  const char* ptr = buffer.data();

  if(buffer.size() < sizeof(RiffChunk))
    return false;

  master = reinterpret_cast<const RiffChunk*>(ptr);
  if(std::memcmp(master->waveId, "WAVE", 4))
    return false;

  for(ptr += sizeof(RiffChunk); ptr <= buffer.data() + buffer.size() - 8;)
  {
    const size_t remainingBytes = buffer.data() + buffer.size() - ptr;

    if(!std::memcmp(ptr, "fmt ", 4) && remainingBytes >= sizeof(FmtChunk))
      format = reinterpret_cast<const FmtChunk*>(ptr);
    else if(!std::memcmp(ptr, "data", 4) && remainingBytes >= sizeof(DataChunk))
      data = reinterpret_cast<const DataChunk*>(ptr);

    const auto chunkSize = *reinterpret_cast<const uint32_t*>(ptr + 4);
    ptr += 4 + 4 + chunkSize;
  }

  if(!format || !data)
    return false;

  const unsigned long remainingDataBytes = buffer.size() - (reinterpret_cast<const char*>(&data->chunkSize) - buffer.data());
  if(data->chunkSize > remainingDataBytes)
  {
    PRINT_ERROR("Wave file data chunk claims it is " << data->chunkSize << " bytes big, but there are only " << remainingDataBytes << " bytes remaining.");
    return false;
  }

  return true;
}

Wave::Wave(File& file)
{
  if(!file.exists())
    return;

  WaveFile waveFile(file);

  if(!waveFile.parse())
  {
    PRINT_ERROR("Could not parse wave file");
    return;
  }

  sampleRate = waveFile.format->sampleRate;
  auto* sampleData = reinterpret_cast<const short*>(&waveFile.data->chunkSize + 1);

  const auto numSamples = waveFile.data->chunkSize / sizeof(short);
  numFrames = numSamples / std::max(static_cast<unsigned short>(1), waveFile.format->numChannels);
  if(waveFile.format->numChannels == 1)
  {
    data.resize(numSamples * 2);
    short* pDst = data.data();
    for(const short* pSrc = sampleData, *pEnd = pSrc + numSamples; pSrc < pEnd; ++pSrc)
    {
      *pDst++ = *pSrc;
      *pDst++ = *pSrc;
    }
  }
  else
  {
    ASSERT(waveFile.format->numChannels == 2);
    data = std::vector<short>(sampleData, sampleData + numSamples);
  }
}

Wave::Wave(const cst_wave* wave)
{
  sampleRate = static_cast<unsigned>(wave->sample_rate);
  numFrames = static_cast<unsigned>(wave->num_samples) / std::max(1, wave->num_channels);

  if(wave->num_channels == 1)
  {
    data.resize(static_cast<unsigned>(wave->num_samples * 2));
    short* pDst = data.data();
    for(const short* pSrc = wave->samples, *pEnd = pSrc + wave->num_samples; pSrc < pEnd; ++pSrc)
    {
      *pDst++ = *pSrc;
      *pDst++ = *pSrc;
    }
  }
  else
  {
    ASSERT(wave->num_channels == 2);
    data = std::vector<short>(wave->samples, wave->samples + wave->num_samples);
  }
}

SoundPlayer::SoundPlayer()
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
    closing.store(true, std::memory_order_relaxed);
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

  while(isRunning() && !closing.load(std::memory_order_relaxed))
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

  playing.store(true, std::memory_order_relaxed);
  VERIFY(!snd_pcm_prepare(soundPlayer.handle));
  while(frames > 0)
  {
    const snd_pcm_uframes_t periodFrames = std::min(frames, soundPlayer.periodSize);
    int ret = static_cast<int>(snd_pcm_writei(soundPlayer.handle, p, periodFrames));
    if(ret < 0)
    {
      ret = snd_pcm_recover(soundPlayer.handle, ret, 0);
      BH_TRACE_MSG(snd_strerror(ret));
    }
    else
    {
      ASSERT(static_cast<snd_pcm_uframes_t>(ret) <= periodFrames);
      p += static_cast<snd_pcm_uframes_t>(ret);
      frames -= static_cast<snd_pcm_uframes_t>(ret);
    }
  }
  snd_pcm_drain(soundPlayer.handle);
  playing.store(false, std::memory_order_relaxed);
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

    if(first.isTextToSpeech && !first.fileOrText.empty())
    {
      const bool isStretched = first.ttsDurationStretchFactor != 1.f;
      const std::string& text = first.fileOrText;
#ifdef TARGET_ROBOT
      fprintf(stderr, "Saying %s\n", text.c_str());
#endif
      if(!first.mute)
      {
        flite_feat_set_float(soundPlayer.voice->features, "duration_stretch", first.ttsDurationStretchFactor);

        if(isStretched)
        {
          cst_wave* rawWave = flite_text_to_wave(text.c_str(), soundPlayer.voice);
          cst_wave_rescale(rawWave, static_cast<int>(textToSpeechVolumeFactor * 65536));
          cst_wave_resample(rawWave, static_cast<int>(sampleRate));
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
            cst_wave_resample(rawWave, static_cast<int>(sampleRate));
            soundInMap = soundPlayer.synthesizedSounds.emplace(std::make_pair(text, Wave(rawWave))).first;
            delete_wave(rawWave);
          }
          soundPlayer.playWave(soundInMap->second);
        }
      }
    }
    else
    {
#ifdef TARGET_ROBOT
      if(first.fileOrText == "DerShredder.wav" && (random() & 1))
        first.fileOrText = "Sabine.wav";
      fprintf(stderr, "Playing %s\n", first.fileOrText.c_str());
#endif
      if(!first.mute)
      {
        File file(soundPlayer.filePrefix + first.fileOrText, "rb");
        if(file.exists())
        {
          Wave wave(file);
          soundPlayer.playWave(wave);
        }
      }
    }
  }
}

int SoundPlayer::enqueue(const SoundRequest& soundRequest)
{
  int queueLength;

  {
    SYNC_WITH(soundPlayer);
    soundPlayer.queue.push_back(soundRequest);
    queueLength = static_cast<int>(soundPlayer.queue.size());
    if(!soundPlayer.started)
    {
      soundPlayer.started = true;
      soundPlayer.start();
    }
    else
      soundPlayer.sem.post();
  }
  return queueLength;
}

int SoundPlayer::play(const std::string& name, bool mute)
{
  const int queueLength = enqueue(SoundRequest(name, mute));
  return mute ? 0 : queueLength;
}

int SoundPlayer::say(const std::string& text, bool mute, float speed)
{
  const float stretchFactor = speed > 0.f ? 1.f / speed : 0.f;
  return enqueue(SoundRequest(text, mute, stretchFactor));
}

bool SoundPlayer::isPlaying()
{
  return soundPlayer.playing.load(std::memory_order_relaxed);
}

#endif
