/**
 * @file AudioProvider.cpp
 * This file implements a module that provides audio samples.
 * @author Thomas Röfer
 * @author Lukas Post
 * @author Lukas Plecher
 */

#include "AudioProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include <type_traits>

MAKE_MODULE(AudioProvider, infrastructure);

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
    if(snd_pcm_open(&handle, allChannels ? "PCH_input" : "default", SND_PCM_STREAM_CAPTURE, 0) >= 0)
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
  VERIFY(!snd_pcm_hw_params_set_rate_near(handle, params, &sampleRate, nullptr));
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

  if(captureVolume != currentCaptureVolume)
  {
    if(!setCaptureVolume(captureCard, captureVolume))
      OUTPUT_WARNING("Could not set capture volume for '" << captureCard << "' to " << captureVolume);
    currentCaptureVolume = captureVolume;
  }

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

  if(onlySoundInSetAndPlaying && theGameInfo.state != STATE_SET && theGameInfo.state != STATE_PLAYING)
    audioData.samples.clear();
}

bool AudioProvider::setCaptureVolume(const std::string& element, float volumePercent)
{
  long min = -1, max = -1;
  const char* card = "default";
  const char* elemName = element.c_str();
  snd_mixer_t *mixer;
  snd_mixer_selem_id_t *sid;

  VERIFY(!snd_mixer_open(&mixer, 0));
  VERIFY(!snd_mixer_attach(mixer, card));
  VERIFY(!snd_mixer_selem_register(mixer, nullptr, nullptr));
  VERIFY(!snd_mixer_load(mixer));

  VERIFY(!snd_mixer_selem_id_malloc(&sid));
  snd_mixer_selem_id_set_index(sid, 0);
  snd_mixer_selem_id_set_name(sid, elemName);

  snd_mixer_elem_t* elem = snd_mixer_find_selem(mixer, sid);
  snd_mixer_selem_id_free(sid);
  if(!elem)
    return false;

  VERIFY(!snd_mixer_selem_get_capture_volume_range(elem, &min, &max));
  ASSERT(min != max);

  // Calculate the actual volume value
  volumePercent = Rangef(0.f, 100.f).limit(volumePercent);
  long volume = static_cast<long>(volumePercent * max / 100.f);

  // Set capture volume for all channels on mixer element
  VERIFY(!snd_mixer_selem_set_capture_volume_all(elem, volume));

  snd_mixer_close(mixer);
  return true;
}

#else // !defined TARGET_ROBOT

AudioProvider::AudioProvider() {}
AudioProvider::~AudioProvider() {}
void AudioProvider::update(AudioData&) {}

#endif
