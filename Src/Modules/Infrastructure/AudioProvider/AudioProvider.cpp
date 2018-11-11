/**
 * @file AudioProvider.cpp
 * This file implements a module that provides audio samples.
 * @author Thomas Röfer
 * @author Lukas Post
 */

#include "AudioProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"

MAKE_MODULE(AudioProvider, cognitionInfrastructure)

#ifdef TARGET_ROBOT

AudioProvider::AudioProvider()
{
  allChannels ? channels = 4 : channels = 2;
  int brokenFirst = (theDamageConfigurationHead.audioChannelsDefect[0] ? 1 : 0) + (theDamageConfigurationHead.audioChannelsDefect[1] ? 1 : 0);
  int brokenSecond = (theDamageConfigurationHead.audioChannelsDefect[2] ? 1 : 0) + (theDamageConfigurationHead.audioChannelsDefect[3] ? 1 : 0);
  unsigned i;
  for(i = 0; i < retries; ++i)
  {
    if(snd_pcm_open(&handle, allChannels ? "4channelsDeinterleaved" : brokenFirst > brokenSecond ? "hw:0,0,1" : "hw:0",
                    snd_pcm_stream_t(SND_PCM_STREAM_CAPTURE | SND_PCM_NONBLOCK), 0) >= 0)
      break;
    Thread::sleep(retryDelay);
  }
  ASSERT(i < retries);

  snd_pcm_hw_params_t* params;
  VERIFY(!snd_pcm_hw_params_malloc(&params));
  VERIFY(!snd_pcm_hw_params_any(handle, params));
  VERIFY(!snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED));
  VERIFY(!snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE));
  VERIFY(!snd_pcm_hw_params_set_rate_near(handle, params, &sampleRate, 0));
  VERIFY(!snd_pcm_hw_params_set_channels(handle, params, channels));
  VERIFY(!snd_pcm_hw_params(handle, params));
  snd_pcm_hw_params_free(params);

  VERIFY(!snd_pcm_prepare(handle));

  ASSERT(channels <= 4);
  short buf[4];
  VERIFY(snd_pcm_readi(handle, buf, 1) >= 0);
}

AudioProvider::~AudioProvider()
{
  snd_pcm_close(handle);
}

void AudioProvider::update(AudioData& audioData)
{
  if(onlySoundInSet && theGameInfo.state != STATE_SET)
    return;

  audioData.channels = channels;
  audioData.sampleRate = sampleRate;

  unsigned available = std::min((unsigned) snd_pcm_avail(handle), maxFrames);
  audioData.samples.resize(available * channels);

  int status = snd_pcm_readi(handle, audioData.samples.data(), available);
  if(status < 0)
  {
    OUTPUT_WARNING("Lost audio stream (" << status << "), recovering...");
    snd_pcm_recover(handle, status, 1);
    ASSERT(channels <= 4);
    short buf[4];
    VERIFY(snd_pcm_readi(handle, buf, 1) >= 0);
    audioData.samples.clear();
  }
}

#else // !defined TARGET_ROBOT

AudioProvider::AudioProvider() {}
AudioProvider::~AudioProvider() {}
void AudioProvider::update(AudioData& audioData) {}

#endif
