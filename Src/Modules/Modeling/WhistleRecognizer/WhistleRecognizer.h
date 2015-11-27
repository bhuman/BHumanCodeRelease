/*
 * @file WhistleRecognizer.h
 *
 * Declaration of module that identifies the sound of a whistle
 *
 * @author Dennis Schuethe
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/Whistle.h"

#ifndef WINDOWS
// We currently do not have a working FFTW3 implementation
// for Windows in our repository.
#include <fftw3.h>

const int WHISTLE_BUFF_LEN = 1024;                  // dataIn
const int WHISTLE_FFT_LEN = (2*WHISTLE_BUFF_LEN);   // fftIn buffer = 2xWHISTLEBUFF_LEN because of zero padding for fast correlation
const int WHISTLE_CORR_LEN = WHISTLE_FFT_LEN;       // length of buffer for the IFFT
const int WHISTLE_OVERLAP = (WHISTLE_BUFF_LEN-256); // make 1/4 of the buffer old data and 3/4 new data (256 of old data)
#endif

MODULE(WhistleRecognizer,
{,
  REQUIRES(AudioData),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(DamageConfigurationHead),
  PROVIDES(Whistle),
  LOADS_PARAMETERS(
  {,
    (float) whistleThreshold,            /**< Minimum correlation for accepting sound as whistle */
    (float) volumeThreshold,             /**< Minimum sound intensity for accepting sound as whistle */
    (int)   timeForOneChannelAcceptance, /**< After having heard the whistle on both channels, this is the amount of time in which one channel is still sufficient */
    (bool)  deactivatePlots,             /**< Unimportant flag */
    (std::string) whistleFile,           /**< Name of the file that stores the reference whistle */
  }),
});

/*
 * @class WhistleRecognizer
 *
 * Module that identifies the sound of a whistle in audio data
 */
class WhistleRecognizer : public WhistleRecognizerBase
{
#ifndef WINDOWS
public:
  /** Constructor */
  WhistleRecognizer();

  /** Destructor */
  ~WhistleRecognizer();

private:
  RingBuffer<float, WHISTLE_BUFF_LEN> inputChannel0; /** Audio data from the first channel */
  RingBuffer<float, WHISTLE_BUFF_LEN> inputChannel1; /** Audio data from the second channel */

  fftw_plan fft2048, ifft2048;   /**< Plans for FFT and inverse FFT of an array of size 2048 */
  double *whistleInput8kHz;      /**< Input data for whistle recognition */
  fftw_complex *fftDataIn;       /**< Output buffer of the FFT / input for inverse FFTW */
  double *corrBuff;              /**< Buffer of the resulting correlation (output of inverse FFTW) */

  float maxAutoCorrelationValue; /**< Maximum value (read from reference whistle file)*/
  int cmpCnt;                    /**< Number of new audio samples */
  uint8_t lastGameState;         /**< Keep last game state for checking state transition to SET */
  unsigned int lastTimeWhistleDetectedInBothChannels; /**< As the name says ... */

  /**
   * Method for recognizing a whistle in one channel
   * @param inputChannel The incoming audio data
   * @param correlation Returns the correlation of the incoming data with the preconfigured reference whistle
   * @return true, if a whistle has been recognized, false otherwise
   */
  bool detectWhistle(const RingBuffer<float, WHISTLE_BUFF_LEN>& inputChannel, double& correlation);

  /**
   * Determines the current average sound level
   */
  float computeCurrentVolume();

  /**
   * Use current sound data for computing and storing a new reference whistle
   */
  void recordNewReferenceWhistle();

  /**
   * Write current data for whistle correlation to a file
   */
  void saveReferenceWhistle();

  /**
   * Load data for whistle correlation to a file
   */
  void loadReferenceWhistle();

#endif

  /**
   * The method that detects the whistle
   * @param Whistle The identified whistle
   */
  void update(Whistle& whistle);
};
