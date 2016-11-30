/*
 * @file NoWirelessSoundRecognizer.h
 *
 * Declaration of a module that identifies different sounds for the No Wireless Challenge.
 *
 * @author Dennis Schuethe
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Representations/Communication/NoWirelessSound.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"

#ifndef WINDOWS
// We currently do not have a working FFTW3 implementation
// for Windows in our repository.
#include <fftw3.h>
#endif

const int SOUND_BUFF_LEN = 1024;                     // number of buffered sound data elements
const int SOUND_FFT_LEN  = (2 * SOUND_BUFF_LEN);     // fftIn buffer = 2xSOUNDBUFF_LEN because of zero padding for fast correlation
const int SOUND_OVERLAP  = (SOUND_BUFF_LEN - 256);   // make 1/4 of the buffer old data and 3/4 new data (256 of old data)

STREAMABLE(Sound,
{,
  (int)(-1) frequency,
  (int)(0) tolerance,
  (float)(-1.f) threshold,
});

MODULE(NoWirelessSoundRecognizer,
{,
  REQUIRES(AudioData),
  REQUIRES(CognitionStateChanges),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(DamageConfigurationHead),
  PROVIDES(NoWirelessSound),
  LOADS_PARAMETERS(
  {,
    (std::vector<Sound>) sounds, /**< The sounds that are checked */
    (float) frequencyVisualizationCut, /**< Do not draw frequencies lower than this threshold */
  }),
});

/*
 * @class NoWirelessSoundRecognizer
 *
 * Module that identifies different sounds in audio data.
 */
class NoWirelessSoundRecognizer : public NoWirelessSoundRecognizerBase
{
#ifndef WINDOWS
public:
  /** Constructor */
  NoWirelessSoundRecognizer();

  /** Destructor */
  ~NoWirelessSoundRecognizer();

private:
  RingBuffer<float, SOUND_BUFF_LEN> inputChannel0;    /**< Audio data from the first channel */
  RingBuffer<float, SOUND_BUFF_LEN> inputChannel1;    /**< Audio data from the second channel */
  fftw_plan fft2048;                                  /**< Plan for FFT of an array of size 2048 */
  double* soundInput8kHz;                             /**< Input data for sound recognition */
  fftw_complex* fftDataIn;                            /**< Output buffer of the FFT / input for inverse FFTW */
  int numberOfNewAudioSamples;                        /**< Number of new audio samples */
  unsigned int lastTimeSoundDetectedInBothChannels;   /**< As the name says ... */
  std::vector<float> soundAmplitudes0;
  std::vector<float> soundAmplitudes1;

  /**
   * Method for recognizing a sound in one channel
   * @param inputChannel The incoming audio data
   * @param amplitudes Writes the result back to this vector
   */
  void checkForSounds(const RingBuffer<float, SOUND_BUFF_LEN>& inputChannel, std::vector<float>& amplitudes);
  
  int getSoundIndex(int frequency);

  /**
   * Determines the current average sound level
   */
  float computeCurrentVolume();

#endif

  /**
   * The method that detects the current sound
   * @param noWirelessSound The identified sound
   */
  void update(NoWirelessSound& noWirelessSound);
};
