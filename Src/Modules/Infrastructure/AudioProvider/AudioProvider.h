/**
 * @file AudioProvider.h
 * This file declares a module that provides audio samples.
 * @author Thomas Röfer
 * @author Lukas Plecher
 */

#pragma once

#ifdef TARGET_ROBOT
#include <alsa/asoundlib.h>
#endif
#include "Framework/Module.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include <string>

MODULE(AudioProvider,
{,
  REQUIRES(GameState),
  REQUIRES(DamageConfigurationHead),
  REQUIRES(AudioData),
  PROVIDES_WITHOUT_MODIFY(AudioData),
  PROVIDES(FrameInfo),
  LOADS_PARAMETERS(
  {,
    (unsigned) retries,              /**< Number of tries to open device. */
    (unsigned) retryDelay,           /**< Delay before a retry to open device. */
    (bool) allChannels,              /**< Use all 4 channels, instead of only two*/
    (unsigned) sampleRate,           /**< Sample rate to capture. This variable will contain the frame rate the driver finally selected. */
    (unsigned) frames,               /**< Number of frames read in one cycle. */
    (bool) onlySoundInSetAndPlaying, /**< If true, the module will not provide audio data in game states other than set and playing. */
    (std::string) captureCard,       /**< The ALSA sound card to be used for audio capture. */
    (float) captureVolume,           /**< The microphone volume in percent. */
  }),
});

class AudioProvider : public AudioProviderBase
{
private:
#ifdef TARGET_ROBOT
  snd_pcm_t* handle;
  int channels;
  float currentCaptureVolume = -1.f;

  bool setCaptureVolume(const std::string& element, float volume);
#endif
  unsigned timestamp = 0;

  void update(AudioData& audioData) override;
  void update(FrameInfo& frameInfo) override {frameInfo.time = timestamp;};

public:
  /**
   * Default constructor.
   */
  AudioProvider();

  /**
   * Destructor.
   */
  ~AudioProvider();
};
