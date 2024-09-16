/**
 * @file WhistleDetector.cpp
 *
 * This file implements a module that identifies the sound of a whistle by
 * combining an approach that is based on a physical model and one that
 * uses a neural network. It is based on the 2023 code release of the
 * Nao Devils.
 *
 * @author Diana Kleingarn
 * @author Dominik Brämer
 * @author Thomas Röfer
 */

#include "WhistleDetector.h"
#include "Debugging/Annotation.h"
#include "Debugging/Plot.h"
#include "Math/Constants.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include <algorithm>

MAKE_MODULE(WhistleDetector);

static DECLARE_SYNC;

WhistleDetector::WhistleDetector()
  : currentFreq(freq),
    detector(&Global::getAsmjitRuntime())
{
  // Load the model.
  model = std::make_unique<NeuralNetworkONNX::Model>(std::string(File::getBHDir()) + "/" + whistleNetPath);
  detector.compile(*model);

  ASSERT(detector.numOfInputs() == 1);
  ASSERT(detector.numOfOutputs() == 1);
  ASSERT(detector.input(0).rank() == 2);
  ASSERT(detector.input(0).dims(1) == 1);

  // Setup buffers for pre- and post-processing.
  amplitudes.resize(detector.input(0).dims(0));
  samples.reserve(amplitudes.size() * 2 - 2);
  thresholdBuffer.reserve(useAdaptiveThreshold ? adaptiveWindowSize : 1);
  nnConfidenceBuffer.reserve(useAdaptiveThreshold ? adaptiveWindowSize / 2 : 1);
  pmConfidenceBuffer.reserve(useAdaptiveThreshold ? adaptiveWindowSize / 2 : 1);

  // Init FFT.
  in = fftw_alloc_real(samples.capacity());
  std::memset(in, 0, sizeof(double) * samples.capacity());
  out = fftw_alloc_complex(amplitudes.size());
  {
    SYNC;
    fft = fftw_plan_dft_r2c_1d(static_cast<int>(samples.capacity()), in, out, FFTW_MEASURE);
  }

  chroma.setResolution(500, static_cast<unsigned>(amplitudes.size()));
}

WhistleDetector::~WhistleDetector()
{
  SYNC;
  fftw_destroy_plan(fft);
  fftw_free(out);
  fftw_free(in);
}

void WhistleDetector::update(Whistle& theWhistle)
{
  DECLARE_PLOT("module:WhistleDetector:confidence");
  DECLARE_PLOT("module:WhistleDetector:confidence:nn");
  DECLARE_PLOT("module:WhistleDetector:confidence:nn:weighted");
  DECLARE_PLOT("module:WhistleDetector:confidence:pm");
  DECLARE_PLOT("module:WhistleDetector:confidence:pm:weighted");
  DECLARE_PLOT("module:WhistleDetector:relLimitCount");
  DECLARE_PLOT("module:WhistleDetector:relLimitCount:weighted");
  DECLARE_PLOT("module:WhistleDetector:threshold");
  DECLARE_PLOT("module:WhistleDetector:amp:mean");
  DECLARE_PLOT("module:WhistleDetector:amp:max");
  DECLARE_DEBUG_RESPONSE("debug images:module:WhistleDetector:fft");
  DECLARE_DEBUG_RESPONSE("debug images:module:WhistleDetector:chroma");

  if(theAudioData.samples.empty())
  {
    // We are currently not recording -> start from scratch once we record again
    detectionCount = 0;
    samples.clear();
    thresholdBuffer.clear();
  }
  else
  {
    size_t sampleIndex = channel;
    while(sampleIndex < theAudioData.samples.size())
    {
      samples.push_front(theAudioData.samples[sampleIndex]);
      sampleIndex += theAudioData.channels;

      if(samples.full())
      {
        // Run whistle detection.
        detect(theWhistle);

        // Drop the older half of the samples.
        while(samples.size() > samples.capacity() / 2)
          samples.pop_back();
      }
    }
  }

  if(theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected) < minTimeBetweenWhistles
     && theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected) >= 0) // For logs
    bestConfidence = 1.f;

  // Confidence has not increased for a while -> report whistle
  if(theFrameInfo.getTimeSince(lastTimeWhistleDetected) >= maxDetectionPause && bestConfidence > 1.f)
  {
    ANNOTATION("WhistleDetector", "Detected with " << static_cast<int>(bestConfidence * 100.f) << "%");
    theWhistle.lastTimeWhistleDetected = lastTimeWhistleDetected;
    theWhistle.channelsUsedForWhistleDetection = numOfChannelsReported;
    theWhistle.confidenceOfLastWhistleDetection = bestConfidence;
    bestConfidence = 1.f;
  }

  DEBUG_RESPONSE_ONCE("module:WhistleDetector:detectNow")
  {
    theWhistle.lastTimeWhistleDetected = theFrameInfo.time;
    theWhistle.channelsUsedForWhistleDetection = numOfChannelsReported;
    theWhistle.confidenceOfLastWhistleDetection = 2.f;
  }

  draw();
}

void WhistleDetector::detect(Whistle& theWhistle)
{
  // Copy to input for FFT.
  STOPWATCH("module:WhistleDetector:samples")
  {
    for(size_t i = 0; i < samples.size(); ++i)
    {
      const float phase = static_cast<float>(pi * i / samples.capacity());
      in[i] = samples[samples.size() - 1 - i]
              * (windowing == hann ? sqr(std::sin(phase))
                 : windowing == nuttall ? 0.355768f - 0.487396f * std::sin(phase)
                                         + 0.144232f * std::sin(2.f * phase)
                                         - 0.012604f * std::sin(3.f * phase)
                 : 0.54f - 0.46f * std::cos(2.f * phase));
    }
  }

  // Run FFT.
  STOPWATCH("module:WhistleDetector:FFT") fftw_execute(fft);

  // These variables are set inside the STOPWATCH, but are also needed outside.
  float relLimitCount;
  auto peak = amplitudes.begin();
  float pmConfidence;
  float currentMaxAmp = 0.f;
  float currentMeanAmp;

  STOPWATCH("module:WhistleDetector:amplitutes")
  {
    float ampSum = 0;
    float limitCount = 0;
    for(size_t i = 0; i < amplitudes.size(); ++i)
    {
      const float amp = static_cast<float>(std::sqrt(sqr(out[i][0]) + sqr(out[i][1])));
      detector.input(0)[i] = 20.f * std::log10(amp);
      amplitudes[i] = amp;
      currentMaxAmp = std::max(currentMaxAmp, amp);
      if(amp > limit)
        ++limitCount;
      ampSum += amp;
    }
    relLimitCount = limitCount / amplitudes.size();
    maxAmpHist.push_front(currentMaxAmp);
    PLOT("module:WhistleDetector:amp:max", currentMaxAmp);
    currentMeanAmp = ampSum / amplitudes.size();
    PLOT("module:WhistleDetector:amp:mean", currentMeanAmp);

    //switch to another mic if the current mic is probably broken
    if(currentMeanAmp >= channelMinAmplitude)
    {
      deafCount = 0;
      theWhistle.channelsUsedForWhistleDetection = numOfChannelsReported;
    }
    else if(++deafCount >= channelMinDeafCount)
    {
      if(channel < static_cast<unsigned int>(theAudioData.channels - 1))
      {
        ++channel;
        deafCount = 0;
      }
      else
      {
        theWhistle.channelsUsedForWhistleDetection = 0;
        if(!allDeafAlert)
        {
          SystemCall::say("All microphones are probably broken.", true);
          allDeafAlert = true;
        }
      }
    }
  }

  STOPWATCH("module:WhistleDetector:detect")
  {
    //do WhistleDetection PM
    const Range<unsigned> pos(static_cast<unsigned>(currentFreq.min * samples.capacity() / theAudioData.sampleRate),
                              static_cast<unsigned>(currentFreq.max * samples.capacity() / theAudioData.sampleRate));

    // find whistle peak between min. freq. position and  max. freq. position
    peak = std::max_element(amplitudes.begin() + pos.min + 1, amplitudes.begin() + pos.max);
    const float ampWeight = useWeightedPMConfidence ? *peak / currentMaxAmp : 1.f;

    //get min/max gradients around peakPos
    const Rangef grad(std::abs(*(peak + 1) - *peak) / currentMaxAmp,
                      std::abs(*peak - * (peak - 1)) / currentMaxAmp);

    pmConfidence = std::min(grad.min, grad.max) * ampWeight;
    if(*peak >= maxAmpHist.average())
    {
      //find first overtone peak between peak freq. position * overtoneMultiplierMin  and peak freq. position * overtoneMultiplierMax
      auto overtone = std::max_element(amplitudes.begin() + std::min(static_cast<unsigned>(amplitudes.size()), 2 * pos.min),
                                       amplitudes.begin() + std::min(static_cast<unsigned>(amplitudes.size()), 2 * pos.max + 1));

      //detect whistle
      if(overtone != amplitudes.end() && *overtone >= currentMeanAmp)
        pmConfidence = std::max(grad.min, grad.max) * ampWeight;
    }

    //do WhistleDetection NN
    detector.apply();
  }

  const float nnConfidence = detector.output(0)[0]; // linear

  //Merge NN, PM and limit information
  if(useAdaptiveThreshold)
    thresholdBuffer.push_front(threshold + (1 - threshold) * limitWeight * relLimitCount);
  else
    thresholdBuffer.push_front(threshold);

  nnConfidenceBuffer.push_front(nnWeight * nnConfidence / (nnWeight + pmWeight) * (1 - limitWeight * relLimitCount));
  pmConfidenceBuffer.push_front(pmWeight * pmConfidence / (nnWeight + pmWeight) * (1 - limitWeight * relLimitCount));
  const float confidence = nnConfidenceBuffer.back() + pmConfidenceBuffer.back();
  const float averageThreshold = thresholdBuffer.average();
  if(thresholdBuffer.full() && confidence > averageThreshold
     && nnConfidenceBuffer.back() > averageThreshold * thresholdRatio
     && pmConfidenceBuffer.back() > averageThreshold * thresholdRatio)  // whistle detected this frame, min of #attack detections needed
  {
    lastTimeCandidateDetected = theFrameInfo.time;
    const unsigned detectedWhistleFrequency = static_cast<unsigned>((peak - amplitudes.begin()) * theAudioData.sampleRate / samples.capacity());
    if(++detectionCount >= minDetections && confidence / averageThreshold > bestConfidence)
    {
      bestConfidence = confidence / averageThreshold;
      lastTimeWhistleDetected = lastTimeCandidateDetected;
    }

    if(freqCalibration && detectedWhistleFrequency > freq.min && detectedWhistleFrequency < freq.max
       && confidence > averageThreshold * 1.5f)
    {
      whistleFreqBuffer.push_front(detectedWhistleFrequency);
      const Rangei diff(std::abs(static_cast<int>(whistleFreqBuffer.average() - currentFreq.min)),
                        std::abs(static_cast<int>(whistleFreqBuffer.average() - currentFreq.max)));
      currentFreq.min = std::min(whistleFreqBuffer.average() - diff.min / 2, whistleFreqBuffer.average() - freqTolerance);
      currentFreq.max = std::max(whistleFreqBuffer.average() + diff.max / 2, whistleFreqBuffer.average() + freqTolerance);
    }
  }
  else if(theFrameInfo.getTimeSince(lastTimeCandidateDetected) >= maxDetectionPause)
  {
    detectionCount = 0;
    lastTimeCandidateDetected = 0; // Allows to rewind logs.
  }

  PLOT("module:WhistleDetector:confidence", confidence);
  PLOT("module:WhistleDetector:confidence:nn", nnConfidence);
  PLOT("module:WhistleDetector:confidence:nn:weighted", nnWeight * nnConfidence);
  PLOT("module:WhistleDetector:confidence:pm", pmConfidence);
  PLOT("module:WhistleDetector:confidence:pm:weighted", pmWeight * pmConfidence);
  PLOT("module:WhistleDetector:relLimitCount", relLimitCount);
  PLOT("module:WhistleDetector:relLimitCount:weighted", limitWeight * relLimitCount);
  PLOT("module:WhistleDetector:threshold", averageThreshold);
}

void WhistleDetector::draw()
{
  //debug draw FFT with detection rects and grids
  COMPLEX_IMAGE("module:WhistleDetector:fft")
  {
    Image<PixelTypes::RGBPixel> fft(512, 300);

    auto drawLine = [&](unsigned x1, unsigned y1, unsigned x2, unsigned y2, PixelTypes::RGBPixel pixel)
    {
      const Geometry::PixeledLine line(x1, y1, x2, y2);
      for(const Vector2i& p : line)
        fft[p.y()][p.x()] = pixel;
    };

    // transform sample rate to fft size to debug image size
    const Range<unsigned> xRange(static_cast<unsigned>(currentFreq.min * samples.capacity() / theAudioData.sampleRate * fft.width / amplitudes.size()),
                                 static_cast<unsigned>(currentFreq.max * samples.capacity() / theAudioData.sampleRate * fft.width / amplitudes.size()));

    // draw main detection rect
    for(unsigned x = xRange.min; x <= xRange.max; ++x)
      drawLine(x, 0, x, fft.height - 1, 0x00ffff);

    // draw horizontal grid
    for(unsigned y = 0; y < fft.height; y += 25)
      drawLine(0, y, fft.width - 1, y, 0x505050);

    unsigned grid = 0;
    for(unsigned x = 0; x < fft.width; ++x)
    {
      // transform FFT size to debug image size
      const unsigned xTemp = static_cast<unsigned>(x * amplitudes.size() / fft.width);
      // remove FFT number errors to avoid possible crashes
      const unsigned yTemp = std::min(fft.height - 1, static_cast<unsigned>(amplitudes[xTemp]));

      // draw vertical grid
      if(xTemp * theAudioData.sampleRate / samples.capacity() >= grid)
      {
        drawLine(x, 0, x, fft.height - 1, 0x505050);
        grid += 1000;
      }

      // draw FFT lines
      drawLine(x, fft.height - yTemp - 1, x, fft.height - 1, 0x0000ff);
    }
    SEND_DEBUG_IMAGE("module:WhistleDetector:fft", fft);
  }

  // draw chroma view of FFTs in a short period of time
  COMPLEX_IMAGE("module:WhistleDetector:chroma")
  {
    const int x = chromaPos;
    chromaPos = (chromaPos + 1) % chroma.width;

    // fill vertical line with FFT information
    for(unsigned y = 0; y < amplitudes.size(); ++y)
    {
      const int brightness = 255 * std::max(0, std::min(50, static_cast<int>(amplitudes[y]))) / 50;
      chroma[amplitudes.size() - y - 1][x] = {0, static_cast<unsigned char>(brightness), 0};
      chroma[amplitudes.size() - y - 1][chromaPos] = 0x0000ff;
    }

    // set counter to next vertical line
    SEND_DEBUG_IMAGE("module:WhistleDetector:chroma", chroma);
  }
}
