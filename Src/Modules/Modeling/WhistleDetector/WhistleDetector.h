/**
 * @file WhistleDetector.h
 *
 * This file declares a module that identifies the sound of a whistle by
 * combining an approach that is based on a physical model and one that
 * uses a neural network. It is based on the 2023 code release of the
 * Nao Devils.
 *
 * @author Diana Kleingarn
 * @author Dominik Brämer
 * @author Thomas Röfer
 */

#pragma once

#include "Debugging/DebugImages.h"
#include "Framework/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Modeling/Whistle.h"
#include "Math/Range.h"
#include "Math/RingBufferWithSum.h"

#include <fftw3.h>
#include <CompiledNN2ONNX/CompiledNN.h>

MODULE(WhistleDetector,
{,
  REQUIRES(FrameInfo),
  REQUIRES(AudioData),
  PROVIDES(Whistle),
  LOADS_PARAMETERS(
  {
    ENUM(Windowing,
    {,
      hamming,
      hann,
      nuttall,
    }),
    (std::string) whistleNetPath, /** The path to the network to load, relative to the B-Human directory. */
    (float) channelMinAmplitude, /** The minimum mean amplitude for a working microphone. */
    (unsigned) channelMinDeafCount, /**< How often must the mean amplitude be too low to switch channel? */
    (Windowing) windowing, /**< The windowing method applied to the input of the FFT. */
    (Range<unsigned>) freq, /**< The frequency range searched for the largest amplitude. */
    (bool) freqCalibration, /**< Is the frequency range adapted to the current whistle? */
    (int) freqTolerance, /**< An additional tolerance around the average whistle frequency. */
    (bool) useWeightedPMConfidence, /**< Is the physical-model-based confidence adapted to the current maximum amplitude? */
    (float) pmWeight, /**< The weight of the physical model's confidence in the overall confidence. */
    (float) nnWeight, /**< The weight of the neural network's confidence in the overall confidence. */
    (float) threshold, /**< The minimum confidence necessary to detect a whistle. */
    (float) thresholdRatio, /**< How much of the threshold must both individual confidences at least reach? */
    (bool) useAdaptiveThreshold,
    (float) limit, /**< The minimum amplitude for being considered in adaptive thresholding. */
    (float) limitWeight, /**< How much is the adaptive threshold influenced by the ratio of loud frequencies? */
    (int) adaptiveWindowSize, /**< The size of buffers for adaptive thresholding. */
    (int) maxDetectionPause, /**< The maximum delay between two detections to belong to the same whistle (in ms). */
    (unsigned) minDetections, /**< The minimum number of detections to accept the whistle. */
    (int) minTimeBetweenWhistles, /**< Minimum time after a whistle was detected to accept the next one (in ms). */
    (unsigned char) numOfChannelsReported, /**< The number of channels reported when listening. */
  }),
});

class WhistleDetector : public WhistleDetectorBase
{
  unsigned channel = 0; /**< The channel, i.e. the microphone, that is used for whistle detection. */
  RingBuffer<float> samples; /**< The audio samples of that channel to process. */

  fftw_plan fft; /**< The plan to compute the FFT. */
  double* in = nullptr; /**< The input of the FFT. */
  fftw_complex* out = nullptr; /**< The output of the FFT. */

  std::vector<float> amplitudes; /**< The amplitudes of the different frequencies. */
  RingBufferWithSum<float, 200> maxAmpHist; /**< The maximum amplitudes of the last 200 FFTs. */
  unsigned deafCount = 0; /**< How often was the mean amplitude too low consecutively on this channel? */
  bool allDeafAlert = false; /**< Already informed that all microphones are broken? */

  Range<unsigned> currentFreq; /**< The frequency window in which it is searched for the whistle. */
  NeuralNetworkONNX::CompiledNN detector; /**< The neural whistle detector. */
  std::unique_ptr<NeuralNetworkONNX::Model> model; /**< The model loaded into the detector. */

  unsigned lastTimeCandidateDetected = 0; /**< The last time an individual detection had a sufficient confidence. */
  unsigned detectionCount = 0; /**< The number of detections that a sufficient confidence for a single whistle. */
  float bestConfidence = 1.f; /**< The highest confidence while detecting the current whistle. */
  unsigned lastTimeWhistleDetected = 0; /**< The time when the highest confidence was reached. */

  RingBufferWithSum<float, 20> thresholdBuffer; /**< The last 10 or only 1 (!useAdaptiveThreshold) confidence thresholds used. */
  RingBufferWithSum<float, 10> nnConfidenceBuffer; /**< The neural network's confidences of the last 10 detections. */
  RingBufferWithSum<float, 10> pmConfidenceBuffer; /**< The physical method's confidences of the last 10 detections. */
  RingBufferWithSum<unsigned, 10> whistleFreqBuffer; /**< The frequencies of the last 10 detected whistles. */

  Image<PixelTypes::RGBPixel> chroma; /**< The amplitudes of the different frequencies (vertical) plotted over time (horizontal). */
  int chromaPos = 0; /**< The column that is drawn next in the image. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theWhistle The representation updated.
   */
  void update(Whistle& theWhistle);

  /**
   * This method performs the actual whistle detection, after enough samples were collected.
   * @param theWhistle Sets the number of channels used to 0 if sound is too quiet or back to 1
   *                   if a whistle was detected.
   */
  void detect(Whistle& theWhistle);

  /** Creates two debug images. */
  void draw();

public:
  WhistleDetector();
  ~WhistleDetector();
};
