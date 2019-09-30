/**
 * @file WhistleRecognizer.h
 *
 * This file declares a module that identifies the sound of a whistle by
 * correlating with a number of templates.
 *
 * @author Tim Laue
 * @author Dennis Schuethe
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Communication/BHumanMessage.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Whistle.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Tools/Streams/Eigen.h"
#include <fftw3.h>

MODULE(WhistleRecognizer,
{,
  USES(GameInfo),
  REQUIRES(AudioData),
  REQUIRES(BHumanMessageOutputGenerator),
  REQUIRES(DamageConfigurationHead),
  REQUIRES(FrameInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(RobotInfo),
  PROVIDES(Whistle),
  LOADS_PARAMETERS(
  {,
    (std::vector<std::string>) whistles, /**< Base names of the files containing the reference whistles. */
    (unsigned) bufferSize, /**< The number of samples buffered per channel. */
    (unsigned) sampleRate, /**< The sample rate actually used. */
    (float) newSampleRatio, /**< The ratio of new samples buffered before recognition is tried again (0..1). */
    (float) minVolume, /**< The minimum volume that must be reached for accepting a whistle [0..1). */
    (float) minCorrelation, /**< The ratio between the selfCorrelation and the current correlation that is accepted ]0..1]. */
    (int) minAnnotationDelay, /**< The minimum time between annotations announcing a detected whistle. */
    (bool) detectInPlaying, /**< Detect whistles in PLAYING state instead of SET state. */
  }),
});

class WhistleRecognizer : public WhistleRecognizerBase
{
  STREAMABLE(Signature,
  {,
    (std::string) name, /**< The name of the whistle. */
    (float)(0.f) selfCorrelation, /**< The self correlation of the spectrum. */
    (std::vector<Vector2d>) spectrum, /**< The spectrum recorded. */
  });

  std::vector<Signature> signatures; /**< All whistle signatures. */
  std::vector<RingBuffer<AudioData::Sample>> buffers; /**< Sample buffers for all channels. */
  bool hasRecorded = false; /**< Was audio recorded in the previous cycle? */
  int samplesRequired = 0; /** The number of new samples required. */
  size_t sampleIndex = 0; /** Index of next sample to process for subsampling. */
  double* samples; /**< The samples after normalization. */
  fftw_complex* spectrum; /**< The spectrum of the samples. */
  double* correlation; /**< The correlation with the signature. */
  fftw_plan fft; /**< The plan to compute the FFT. */
  fftw_plan ifft; /**< The plan to compute the inverse FFT. */
  float bestCorrelation = 1.f; /**< The best correlation since the last network packet was sent twice. */
  bool bestUpdated = false; /**< Was the best correlation updated since the last network packet was sent? */
  Image<PixelTypes::Edge2Pixel> canvas; /**< Canvas for drawing spectra. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theWhistle The representation updated.
   */
  void update(Whistle& theWhistle) override;

  /**
   * Correlate the samples recorded with a signature spectrum.
   * @param signature The spectrum of a recorded whistle.
   * @param buffer The samples recorded.
   * @param record Record into the parameter "signature" instead of correlating with it.
   * @return The correlation between signature and buffer. 0 if the volume was too low.
   */
  float correlate(std::vector<Vector2d>& signature, const RingBuffer<AudioData::Sample>& buffer,
                  bool record = false);

public:
  WhistleRecognizer();
  ~WhistleRecognizer();
};
