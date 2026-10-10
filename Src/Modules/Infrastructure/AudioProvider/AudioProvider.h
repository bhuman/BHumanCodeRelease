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
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include <string>

#ifdef TARGET_ROBOT
#include <WakeupApi.h>
#endif

MODULE(AudioProvider,
{,
  USES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(AudioData),
  PROVIDES_WITHOUT_MODIFY(AudioData),
  PROVIDES(FrameInfo),
  LOADS_PARAMETERS(
  {
    ENUM(CaptureMode,
    {,
      always,
      onlyInSet,
      onlyInSetAndPlaying,
    }),

    (std::string) card,              /**< The device opened for capture. */
    (std::string) element,           /**< The device element to configure the capture volume. */
    (unsigned) channels,             /**< The number of channels to capture. */
    (unsigned) sampleRate,           /**< Sample rate to capture. This variable will contain the frame rate the driver finally selected. */
    (unsigned) frames,               /**< Number of frames to read in one cycle. */
    (unsigned) captureVolume,        /**< The microphone volume in percent. */
    (unsigned) retries,              /**< Number of tries to open device. */
    (unsigned) retryDelay,           /**< Delay before a retry to open device. */
    (CaptureMode) captureMode,       /**< In which game states is audio captured? */
    (bool) duerWenEncoded,           /**< Is the input DuerWen encoded? */
    (int) audioLostSoundTime,        /**< Wait this long between sound warning, when audio is lost */
  }),
});

class AudioProvider : public AudioProviderBase
{
private:
#ifdef TARGET_ROBOT
  snd_pcm_t* handle; /**< The ALSA handle for the audio input. */
  unsigned currentCaptureVolume = 1000; /**< The current capture volume. The initial value enforces setting the volume at least once. */
  unsigned soundLostTimestamp = 0; /**< The last time a lost audio stream was announced. */
  long captured = 0; /**< The number of frames captured so far. */

  /** Sets the capture volume to \c captureVolume . */
  bool setCaptureVolume();
#endif

#ifdef TARGET_ROBOT
  HWWakeup wakeupHandle = nullptr; /**< The wakeup handle. */
  std::vector<short> buffers[8]; /**< Buffers for DuerWen decoding. */

  /**
   * Decodes DuerWen encoded audio data.
   * @param samples As input, the samples are interpreted as frames of 6 channels (short ints).
   *                As output, these are replaced by frames of three channels (floats).
   */
  void duerWenDecode(std::vector<AudioData::Sample>& samples);
#endif

  unsigned timestamp = 0; /**< The time when the last block of frames was captured (in ms).*/

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theAudioData The representation updated.
   */
  void update(AudioData& theAudioData) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theFrameInfo The representation updated.
   */
  void update(FrameInfo& theFrameInfo) override {theFrameInfo.time = timestamp;};

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
