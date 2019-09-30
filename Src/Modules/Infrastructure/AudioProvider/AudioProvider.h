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
#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/AudioData.h"

MODULE(AudioProvider,
{,
  USES(GameInfo),
  REQUIRES(DamageConfigurationHead),
  REQUIRES(RawGameInfo),
  PROVIDES_WITHOUT_MODIFY(AudioData),
  LOADS_PARAMETERS(
  {,
    (unsigned) retries,    /**< Number of tries to open device. */
    (unsigned) retryDelay, /**< Delay before a retry to open device. */
    (bool) allChannels,    /**< Use all 4 channels, instead of only two*/
    (unsigned) sampleRate, /**< Sample rate to capture. This variable will contain the framerate the driver finally selected. */
    (unsigned) maxFrames,  /**< Maximum number of frames read in one cycle. */
    (bool) onlySoundInSet, /**< If true, the module will not provide audio data in game states other than set */
  }),
});

class AudioProvider : public AudioProviderBase
{
private:
#ifdef TARGET_ROBOT
  snd_pcm_t* handle;
  int channels;
#endif
  void update(AudioData& audioData) override;

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
