/*
 * @file WhistleRecognizer.h
 *
 * Declaration of module that identifies the sound of a whistle.
 * It is possible to compare the current sound to multiple reference whistles.
 *
 * @author Dennis Schuethe
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Modeling/Whistle.h"

#include <queue>
#include <fftw3.h>

const int WHISTLE_BUFF_LEN = 1024;                   // number of buffered sound data elements
const int WHISTLE_FFT_LEN  = (2 * WHISTLE_BUFF_LEN); // fftIn buffer = 2xWHISTLEBUFF_LEN because of zero padding for fast correlation
const int WHISTLE_CORR_LEN = WHISTLE_FFT_LEN;        // length of buffer for the IFFT
const int WHISTLE_OVERLAP  = (WHISTLE_BUFF_LEN - 256); // make 1/4 of the buffer old data and 3/4 new data (256 of old data)

MODULE(WhistleRecognizer,
{,
  REQUIRES(AudioData),
  REQUIRES(CognitionStateChanges),
  REQUIRES(DamageConfigurationHead),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(RobotInfo),
  PROVIDES(Whistle),
  LOADS_PARAMETERS(
  {,
    (std::vector<float>) whistleThresholds,    /**< Minimum correlations for accepting a particular sound as whistle */
    (float) volumeThreshold,                   /**< Minimum sound intensity for accepting sound as whistle */
    (int)   timeForOneChannelAcceptance,       /**< After having heard the whistle on both channels, this is the amount of time in which one channel is still sufficient */
    (bool)  deactivatePlots,                   /**< Unimportant flag */
    (std::vector<std::string>) whistleFiles,   /**< Name of the files that store the reference whistles */
    (std::string) nameForNewWhistles,          /**< Name of the file that is used for saving a new whistle */
    (int)   soundReplayDelay,                  /**< Delay after events that resulted in replaying sounds (in ms). */
    (unsigned) minConsecutiveWhistles,         /**< Number of times a Whistle should be heard in a period until it is considered a whistle */
    (int) whistleCombinationPeriod,            /**<  length of the period, wehere a specific number of whistles should be heard*/
  }),
});

/*
 * @class WhistleRecognizer
 *
 * Module that identifies the sound of a whistle in audio data.
 */
class WhistleRecognizer : public WhistleRecognizerBase
{
public:
  /** Constructor */
  WhistleRecognizer();

  /** Destructor */
  ~WhistleRecognizer();

private:
  RingBuffer<float, WHISTLE_BUFF_LEN> inputChannel0;  /**< Audio data from the first channel */
  RingBuffer<float, WHISTLE_BUFF_LEN> inputChannel1;  /**< Audio data from the second channel */
  fftw_plan fft2048, ifft2048;                        /**< Plans for FFT and inverse FFT of an array of size 2048 */
  double* whistleInput8kHz;                           /**< Input data for whistle recognition */
  fftw_complex* fftDataIn;                            /**< Output buffer of the FFT / input for inverse FFTW */
  double* corrBuff;                                   /**< Buffer of the resulting correlation (output of inverse FFTW) */
  std::vector<fftw_complex*> fftCmpData;              /**< Reference whistles */
  fftw_complex* newfftCmpData;                        /**< Buffer for storing a new whistle */
  std::vector<float> maxAutoCorrelationValue;         /**< Maximum values (read from reference whistle files)*/
  float newMaxAutoCorrelationValue;                   /**< Maximum value of new whistle */
  int numberOfNewAudioSamples;                        /**< Number of new audio samples */
  unsigned int lastTimeWhistleDetectedInBothChannels; /**< As the name says ... */
  unsigned int lastTimefft2048Computed;               /**< As the name says ... */
  bool audioChannel0Defect = false;                   /**< Is the first channel used defect? */
  bool audioChannel1Defect = false;                   /**< Is the second channel used defect? */
  unsigned lastTimeWithReplayEvent = 0;               /**< For avoiding detection of sound replayed */
  Whistle currentWhistle;
  std::queue<Whistle> lastDetectedWhistles;

  /**
   * Method for recognizing a whistle in one channel
   * @param inputChannel The incoming audio data
   * @param whistleNumber The whistle to correlate with
   * @param correlation Returns the correlation of the incoming data with the preconfigured reference whistle
   * @return The match with the correlation limit in percent (can be higher than 100 %
   */
  float getWhistleCorrelationInPercent(const RingBuffer<float, WHISTLE_BUFF_LEN>& inputChannel, unsigned whistleNumber);

  /**
   * Method searches for value higher than 100 and returns the index of element with the highest value.
   * If no element is larger than 100, the method returns -1
   * @param correlationList A list of numbers
   * @param bestCorrelation The value of the highest correlation
   * @return -1, if no value is higher or equal to 100, the index of the highest value otherwise.
   */
  int listContainsValueHigherThan100(const std::vector<float>& correlationList, float& bestCorrelation);

  /**
   * Output on console, for debugging and configuration
   * @param channelNumer The number of the channel (0/1)
   * @param correlationList A list of correlations in percent
   */
  void printCorrelationResults(int channelNumber, const std::vector<float>& correlationList);

  /**
   * Determines the current average sound level
   */
  float computeCurrentVolume();

  /**
   * Use current sound data for computing and storing a new reference whistle
   */
  void recordNewReferenceWhistle();

  /**
   * Write the last recorded reference whistle to a file
   */
  void saveReferenceWhistle();

  /**
   * Load data for whistle correlation from a file
   * @param loadCmpData The memory buffer to which the data will be written
   * @param maxAutoCorrelationValue The correlation value for the reference data
   * @param fileName The name of the file which contains the rerefence data
   */
  void loadReferenceWhistle(fftw_complex* loadCmpData, float& maxAutoCorrelationValue, const std::string& fileName);

  /**
   * The method that detects the whistle
   * @param Whistle The identified whistle
   */
  void update(Whistle& whistle);
};
