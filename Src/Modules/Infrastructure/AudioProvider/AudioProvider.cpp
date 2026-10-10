/**
 * @file AudioProvider.cpp
 * This file implements a module that provides audio samples.
 * @author Thomas Röfer
 * @author Lukas Post
 * @author Lukas Plecher
 */

#include "AudioProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Platform/Thread.h"
#include <type_traits>

MAKE_MODULE(AudioProvider);

#ifdef TARGET_ROBOT

AudioProvider::AudioProvider()
{
  // The implementation relies on 6 short int channels fitting in the space
  // of 3 float channels. This is only required when reading DuerWen encoded
  // audio data provided by the K1 microphone array.
  ASSERT(!duerWenEncoded || (std::is_same<AudioData::Sample, float>::value));
  ASSERT(!duerWenEncoded || channels == 3);
  ASSERT(!duerWenEncoded || frames == 1024);

  if(duerWenEncoded)
  {
    VERIFY(!Duerwen_wakeup_init(&wakeupHandle, 0));
    for(auto& buffer : buffers)
      buffer.resize(frames);
  }

  unsigned i;
  for(i = 0; i < retries; ++i)
  {
    if(snd_pcm_open(&handle, card.c_str(), SND_PCM_STREAM_CAPTURE, 0) >= 0)
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
  VERIFY(!snd_pcm_hw_params_set_format(handle, params, std::is_same<AudioData::Sample, short>::value || duerWenEncoded
                                       ? SND_PCM_FORMAT_S16_LE : SND_PCM_FORMAT_FLOAT_LE));
  VERIFY(!snd_pcm_hw_params_set_rate_near(handle, params, &sampleRate, nullptr));
  VERIFY(!snd_pcm_hw_params_set_channels(handle, params, duerWenEncoded ? channels * 2 : channels));
  VERIFY(!snd_pcm_hw_params(handle, params));
  snd_pcm_hw_params_free(params);

  VERIFY(!snd_pcm_prepare(handle));

  const_cast<AudioData&>(theAudioData).samples.resize(channels);
  captured = std::max(snd_pcm_readi(handle, const_cast<AudioData&>(theAudioData).samples.data(), 1), 0l);
  ASSERT(captured == 1);
}

AudioProvider::~AudioProvider()
{
  snd_pcm_close(handle);
  if(wakeupHandle)
    Duerwen_wakeup_unit(wakeupHandle);
}

void AudioProvider::update(AudioData& theAudioData)
{
  theAudioData.channels = channels;
  theAudioData.sampleRate = sampleRate;

  if(captureVolume != currentCaptureVolume)
  {
    if(!setCaptureVolume())
      OUTPUT_WARNING("Could not set capture volume for '" << element << "' to " << captureVolume);
    currentCaptureVolume = captureVolume;
  }

  theAudioData.samples.resize(frames * channels);
  for(;;)
  {
    const long available = std::min(snd_pcm_avail(handle), frames - captured);
    if(available > 0)
    {
      const int status = static_cast<int>(snd_pcm_readi(handle, theAudioData.samples.data() + captured * channels, available));
      if(status < 0)
      {
        OUTPUT_WARNING("Lost audio stream (" << status << "), recovering...");
        snd_pcm_recover(handle, status, 1);
        theAudioData.samples.resize(channels);
        captured = std::max(snd_pcm_readi(handle, theAudioData.samples.data(), 1), 0l);
        if(captured != 1 && theFrameInfo.getTimeSince(soundLostTimestamp) > audioLostSoundTime)
        {
          soundLostTimestamp = theFrameInfo.time;
          SystemCall::say("Lost audio stream!");
        }
        theAudioData.samples.resize(captured * channels);
        if(!captured)
          break;
      }
      else
        captured += available;
      if(captured == frames)
        break;
    }
    Thread::sleep(1 + (frames - static_cast<unsigned>(captured)) * 1000 / sampleRate);
  }

  // Set timestamp after recording to keep semantics of previous approach.
  timestamp = Time::getCurrentSystemTime();

  // Drop recordings in game state where they are not needed.
  switch(captureMode)
  {
    case onlyInSet:
      if(!theGameState.isSet())
        theAudioData.samples.clear();
      break;
    case onlyInSetAndPlaying:
      if(!theGameState.isSet() && !theGameState.isPlaying())
        theAudioData.samples.clear();
  }

  if(duerWenEncoded && !theAudioData.samples.empty())
    duerWenDecode(theAudioData.samples);

  // Reset for next capture.
  captured = 0;
}

bool AudioProvider::setCaptureVolume()
{
  snd_mixer_t* mixer;
  VERIFY(!snd_mixer_open(&mixer, 0));
  VERIFY(!snd_mixer_attach(mixer, card.c_str()));
  VERIFY(!snd_mixer_selem_register(mixer, nullptr, nullptr));
  VERIFY(!snd_mixer_load(mixer));

  snd_mixer_selem_id_t* sid;
  VERIFY(!snd_mixer_selem_id_malloc(&sid));
  snd_mixer_selem_id_set_index(sid, 0);
  snd_mixer_selem_id_set_name(sid, element.c_str());

  snd_mixer_elem_t* elem = snd_mixer_find_selem(mixer, sid);
  snd_mixer_selem_id_free(sid);
  if(!elem)
    return false;

  long min = -1, max = -1;
  VERIFY(!snd_mixer_selem_get_capture_volume_range(elem, &min, &max));
  ASSERT(min != max);

  // Calculate the actual volume value
  const long volume = static_cast<long>(std::min(100u, captureVolume) * max / 100);

  // Set capture volume for all channels on mixer element
  VERIFY(!snd_mixer_selem_set_capture_volume_all(elem, volume));

  snd_mixer_close(mixer);
  return true;
}

void AudioProvider::duerWenDecode([[maybe_unused]] std::vector<AudioData::Sample>& samples)
{
  const std::array<short, 6>* encoded = reinterpret_cast<const std::array<short, 6>*>(samples.data());
  for(unsigned sample = 0; sample < frames; ++sample)
    for(int channel = 0; channel < 4; ++channel)
      buffers[channel][sample] = encoded[sample][channel];
  VERIFY(!Duerwen_wakeup_three_write_data(wakeupHandle, buffers[0].data(), buffers[1].data(), buffers[2].data(), buffers[3].data(),
                                          buffers[4].data(), buffers[5].data(), buffers[6].data(), buffers[7].data()));
  samples.clear();
  static const float factor = 1.f / -static_cast<float>(std::numeric_limits<short>::min());
  for(unsigned sample = 0; sample < frames; ++sample)
    for(unsigned channel = 0; channel < channels; ++channel)
      samples.push_back(static_cast<float>(buffers[channel][sample]) * factor);
}


#else // !defined TARGET_ROBOT

AudioProvider::AudioProvider() {}
AudioProvider::~AudioProvider() {}
void AudioProvider::update(AudioData&)
{
  Thread::yield();
  timestamp = Time::getCurrentSystemTime();
}

#endif
