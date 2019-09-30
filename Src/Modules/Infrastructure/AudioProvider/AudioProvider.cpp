/**
 * @file AudioProvider.cpp
 * This file implements a module that provides audio samples.
 * @author Thomas Röfer
 * @author Lukas Post
 */

#include "AudioProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include <type_traits>

MAKE_MODULE(AudioProvider, infrastructure)

#ifdef TARGET_ROBOT

AudioProvider::AudioProvider()
{
  allChannels ? channels = 4 : channels = 2;
  int brokenFirst = (theDamageConfigurationHead.audioChannelsDefect[0] ? 1 : 0) + (theDamageConfigurationHead.audioChannelsDefect[1] ? 1 : 0);
  int brokenSecond = (theDamageConfigurationHead.audioChannelsDefect[2] ? 1 : 0) + (theDamageConfigurationHead.audioChannelsDefect[3] ? 1 : 0);
  unsigned i;
  for(i = 0; i < retries; ++i)
  {
    if(brokenFirst > brokenSecond)
      FAIL("Cannot handle broken microphones");
    if(snd_pcm_open(&handle, allChannels ? "PCH_input" : "default", snd_pcm_stream_t(SND_PCM_STREAM_CAPTURE | SND_PCM_NONBLOCK), 0) >= 0)
      break;
    Thread::sleep(retryDelay);
  }
  ASSERT(i < retries);

  snd_pcm_hw_params_t* params;
  VERIFY(!snd_pcm_hw_params_malloc(&params));
  VERIFY(snd_pcm_hw_params_any(handle, params) >= 0);
  VERIFY(!snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED));
  static_assert(std::is_same<AudioData::Sample, short>::value
                || std::is_same<AudioData::Sample, float>::value, "Wrong audio sample type");
  VERIFY(!snd_pcm_hw_params_set_format(handle, params, std::is_same<AudioData::Sample, short>::value
                                       ? SND_PCM_FORMAT_S16_LE : SND_PCM_FORMAT_FLOAT_LE));
  VERIFY(!snd_pcm_hw_params_set_rate_near(handle, params, &sampleRate, 0));
  VERIFY(!snd_pcm_hw_params_set_channels(handle, params, channels));
  VERIFY(!snd_pcm_hw_params(handle, params));
  snd_pcm_hw_params_free(params);

  VERIFY(!snd_pcm_prepare(handle));

  ASSERT(channels <= 4);
  AudioData::Sample buf[4];
  VERIFY(snd_pcm_readi(handle, buf, 1) >= 0);
}

AudioProvider::~AudioProvider()
{
  snd_pcm_close(handle);
}

void AudioProvider::update(AudioData& audioData)
{
  audioData.channels = channels;
  audioData.sampleRate = sampleRate;

  unsigned available = std::min(static_cast<unsigned>(snd_pcm_avail(handle)), maxFrames);
  audioData.samples.resize(available * channels);

  int status = static_cast<int>(snd_pcm_readi(handle, audioData.samples.data(), available));
  if(status < 0)
  {
    OUTPUT_WARNING("Lost audio stream (" << status << "), recovering...");
    snd_pcm_recover(handle, status, 1);
    ASSERT(channels <= 4);
    AudioData::Sample buf[4];
    VERIFY(snd_pcm_readi(handle, buf, 1) >= 0);
    audioData.samples.clear();
  }

  if(onlySoundInSet && (theRawGameInfo.state != STATE_SET || theGameInfo.state == STATE_PLAYING))
    audioData.samples.clear();
}

#else // !defined TARGET_ROBOT

AudioProvider::AudioProvider() {}
AudioProvider::~AudioProvider() {}
void AudioProvider::update(AudioData& audioData) {}

#endif
