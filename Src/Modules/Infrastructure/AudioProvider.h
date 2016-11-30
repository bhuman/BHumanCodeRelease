/**
 * @file AudioProvider.h
 * This file declares a module that provides audio samples.
 * @author Thomas Röfer
 */

#pragma once

#ifdef TARGET_ROBOT
#include <alsa/asoundlib.h>
#endif
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/GameInfo.h"

MODULE(AudioProvider,
{,
  REQUIRES(GameInfo),
  PROVIDES_WITHOUT_MODIFY(AudioData),
  LOADS_PARAMETERS(
  {,
    (unsigned)(10) retries,      /**< Number of tries to open device. */
    (unsigned)(500) retryDelay,  /**< Delay before a retry to open device. */
    (unsigned)(2) channels,      /**< Number of channels to capture. */
    (unsigned)(8000) sampleRate, /**< Sample rate to capture. This variable will contain the framerate the driver finally selected. */
    (unsigned)(5000) maxFrames,  /**< Maximum number of frames read in one cycle. */
    (bool)(true) onlySoundInSet, /**< If true, the module will not provide audio data in game states other than set */
  }),
});

class AudioProvider : public AudioProviderBase
{
private:
#ifdef TARGET_ROBOT
  snd_pcm_t* handle;
#endif
  void update(AudioData& audioData);

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
