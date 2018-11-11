/*
 * @file WhistleRecognizer112.h
 *
 * Declaration of module that identifies the sound of a whistle.
 * It is possible to compare the current sound to multiple reference whistles.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author Dennis Schuethe
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Tools/Streams/AutoStreamable.h"
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

const int WHISTLE_BUFF_LEN = 1024;                     // number of buffered sound data elements
const int WHISTLE_FFT_LEN  = (2 * WHISTLE_BUFF_LEN);   // fftIn buffer = 2xWHISTLEBUFF_LEN because of zero padding for fast correlation
const int WHISTLE_CORR_LEN = WHISTLE_FFT_LEN;          // length of buffer for the IFFT
const int WHISTLE_OVERLAP  = (WHISTLE_BUFF_LEN - 256); // make 1/4 of the buffer old data and 3/4 new data (256 of old data)

MODULE(WhistleRecognizer112,
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
    (float) volumeThreshold,                   /**< Minimum sound intensity for accepting sound as whistle */
    (int)   timeForOneChannelAcceptance,       /**< After having heard the whistle on both channels, this is the amount of time in which one channel is still sufficient */
    (bool)  deactivatePlots,                   /**< Unimportant flag */
    (std::vector<std::string>) whistleFiles,   /**< Name of the files that store the reference whistles */
    (std::string) nameForNewWhistles,          /**< Name of the file that is used for saving a new whistle */
    (int)   soundReplayDelay,                  /**< Delay after events that resulted in replaying sounds (in ms). */
    (unsigned) minConsecutiveWhistles,         /**< Number of times a whistle should be heard within a given period of time until it is considered a whistle */
    (int) whistleCombinationPeriod,            /**< Length of the period, wehere a specific number of whistles should be heard*/
  }),
});

/*
 * @class WhistleRecognizer112
 *
 * Module that identifies the sound of a whistle in audio data.
 */
class WhistleRecognizer112 : public WhistleRecognizer112Base
{
  /**
   * @class WhistleSignature
   *
   * Information about a prerecorded whistle: correlation data as well as the associated thresholds
   */
  STREAMABLE(WhistleSignature,
  {
    fftw_complex* fftCmpData;       /**< Buffer for storing correlation data, not streamed */
    
    /** Default constructor */
    WhistleSignature() {
      fftCmpData = 0;
    }
    
    /** Constructor
     * @param fileName Name of the associated whistle signature file
     * @param thresh Threshold for detection (the lower the threshold, the more likely a detection)
     * @param loadFile Load data from file, if true. Leave correlation data emtpy, otherwise
     */
    WhistleSignature(const std::string& fileName, float thresh = 0.9f, bool loadFile = true): fileName(fileName)
    {
      threshold = thresh;
      active = true;
      fftCmpData = new fftw_complex[WHISTLE_BUFF_LEN + 1];
      if(loadFile) // Load an existing signature
      {
        InBinaryFile file(std::string("Whistles112/") + fileName + ".dat");
        ASSERT(file.exists());
        for(int i = 0; i < WHISTLE_BUFF_LEN + 1; i++)
        {
          file >> fftCmpData[i][0];
          file >> fftCmpData[i][1];
        }
        file >> maxAutoCorrelationValue;
        file >> threshold;
      }
    }
    
    /** For some reasons, the destructor did not work in combination with vector.
     *  Thus, this method should be called, if the signature is not needed anymore
     */
    void freeFFTWSTuff()
    {
      if(fftCmpData)
        delete [] fftCmpData;
    }
    
    /** Write all signatures and thresholds to the robot's disk */
    void save() const
    {
      OutBinaryFile file(std::string("Whistles112/") + fileName + ".dat");
      for(int i = 0; i < WHISTLE_BUFF_LEN + 1; i++)
      {
        file << fftCmpData[i][0];
        file << fftCmpData[i][1];
      }
      file << maxAutoCorrelationValue;
      file << threshold;
      OUTPUT_TEXT("Whistle has been saved to " << fileName << ".dat.");
    },
    
    (std::string)("") fileName,            /**< The name of the file to stored this signature's information */
    (bool)(true) active,                   /**< All whistles are active by default. For testing purposes, this flag can be set to false and the whistle becomes ignored. */
    (float)(0.f) threshold,                /**< To recognize a whistle, (current correlation value / maxAutoCorrelationValue) must be larger than this threshold. */
    (float)(1.f) maxAutoCorrelationValue,  /**< Maximum value of the correlation (during recording) */
  });
  
  /*
   * @class WhistleSignatures
   * Workaround for streaming
   */
  STREAMABLE(WhistleSignatures,
  {,
    (std::vector<WhistleSignature>) list,  /**< List of all signatures */
  });
  
public:
  /** Module constructor */
  WhistleRecognizer112();

  /** Module destructor */
  ~WhistleRecognizer112();

private:
  RingBuffer<float, WHISTLE_BUFF_LEN> inputChannel0;  /**< Audio data from the first channel */
  RingBuffer<float, WHISTLE_BUFF_LEN> inputChannel1;  /**< Audio data from the second channel */
  fftw_plan fft2048, ifft2048;                        /**< Plans for FFT and inverse FFT of an array of size 2048 */
  double* whistleInput8kHz;                           /**< Input data for whistle recognition */
  fftw_complex* fftDataIn;                            /**< Output buffer of the FFT / input for inverse FFTW */
  double* corrBuff;                                   /**< Buffer of the resulting correlation (output of inverse FFTW) */
  WhistleSignatures whistleSignatures;                /**< Data about whistles that are compared to the current audio signal */
  int numberOfNewAudioSamples;                        /**< Number of new audio samples */
  unsigned int lastTimeWhistleDetectedInBothChannels; /**< As the name says ... */
  unsigned int lastTimefft2048Computed;               /**< As the name says ... */
  bool audioChannel0Defect = false;                   /**< Is the first channel used defect? */
  bool audioChannel1Defect = false;                   /**< Is the second channel used defect? */
  unsigned lastTimeWithReplayEvent = 0;               /**< For avoiding detection of sound replayed */
  Whistle currentWhistle;                             /**< Added by a student and thus not documented, of course ... */
  std::queue<Whistle> lastDetectedWhistles;           /**< Added by a student and thus not documented, of course ... */

  /**
   * Method for recognizing a whistle in one channel
   * @param inputChannel The incoming audio data
   * @param whistleSignature One whistle to compare the current audio data with
   * @return The match with the correlation limit in percent (can be higher than 100 %)
   */
  float getWhistleCorrelationInPercent(const RingBuffer<float, WHISTLE_BUFF_LEN>& inputChannel, const WhistleSignature& whistleSignature);

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
   * Write all currently active whistle signatures to disk (on robot).
   */
  void saveWhistleSignatures();

  /**
   * The method that detects the whistle
   * @param Whistle The identified whistle
   */
  void update(Whistle& whistle) override;
};
