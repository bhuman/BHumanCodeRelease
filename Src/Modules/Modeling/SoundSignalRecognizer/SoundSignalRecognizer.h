/*
 * @file SoundSignalRecognizer.h
 *
 * Declaration of module that identifies audio signals for the
 * Sound Recognition Challenge 2014
 *
 * @author Dennis Schuethe
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#ifndef WINDOWS

#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Modeling/SoundSignal.h"
#include <fftw3.h>

const int INPUT_BUFFER_LENGTH   = 100;
const int DFT_LENGTH            = 100;
const int LOWPASS_BUFFER_LENGTH = 30;
const int AVG_FILTER_LEN        = 10;
const int NUM_OF_AVG_VALUES     = 45;
const int OVERLAP = LOWPASS_BUFFER_LENGTH - 20;
const int REAL = 0;     // real value of complex
const int IMAG = 1;     // imaginary value of complex
const int Fslow = 1000; // downsampled frequency
const int F320 = (320 * DFT_LENGTH) / Fslow; // the value where the 320Hz signal is located in the FFT
const int F200 = (200 * DFT_LENGTH) / Fslow; // the value where the 320Hz signal is located in the FFT

const int WHISTLE_BUFF_LEN = 1024;                 // dataIn
const int  WHISTLE_FFT_LEN = (2*WHISTLE_BUFF_LEN);  // fftIn buffer = 2xWHISTLEBUFF_LEN because of zero padding for fast correlation
const int  WHISTLE_CORR_LEN = WHISTLE_FFT_LEN;      // length of buffer for the IFFT
const int WHISTLE_OVERLAP = (WHISTLE_BUFF_LEN-256); // make 1/4 of the buffer old data and 3/4 new data (256 of old data)
const float MAX_AC_VALUE = 1522544.77f;                 // max value of auto correlation of comparative signal

MODULE(SoundSignalRecognizer,
{,
  REQUIRES(FrameInfo),
  REQUIRES(AudioData),
  PROVIDES_WITH_MODIFY(SoundSignal),
  LOADS_PARAMETERS(
  {,
    (float) whistleThreshold,    /**< Minimum correlation for accepting sound as whistle */
    (bool)  deactivatePlots,
    (bool)  whistleNeedsToBeInBothChannels,
    (float) afskSignalStrengthThreshold,
    (bool)  disableSignalRecognition,
    (bool)  disableWhistleRecognition,
    (int)   minPeakDistanceInBuffer,
    (int)   maxPeakDistanceInBuffer,
    (int)   minNumOfPeaks,
    (int)   maxNumOfPeaks,
  }),
});

/*
 * @class SoundSignalRecognizer
 *
 * Module that identifies signals in audio data
 */
class SoundSignalRecognizer : public SoundSignalRecognizerBase
{
  RingBuffer<float, WHISTLE_BUFF_LEN> inputChannel0;
  RingBuffer<float, WHISTLE_BUFF_LEN> inputChannel1;
  RingBuffer<float, LOWPASS_BUFFER_LENGTH> lowPassOutput0;
  RingBuffer<float, LOWPASS_BUFFER_LENGTH> lowPassOutput1;
  RingBuffer<double, AVG_FILTER_LEN> avgFilter320Channel0; // average filter for 320 Hz signals in spectrum
  RingBuffer<double, AVG_FILTER_LEN> avgFilter320Channel1; // average filter for 320 Hz signals in spectrum
  RingBuffer<double, NUM_OF_AVG_VALUES> avgValues320Channel0; // values for peak detection
  RingBuffer<double, NUM_OF_AVG_VALUES> avgValues320Channel1; // values for peack detection
  RingBuffer<double, AVG_FILTER_LEN> avgFilter200Channel0; // average filter for 200 Hz signals in spectrum
  RingBuffer<double, AVG_FILTER_LEN> avgFilter200Channel1; // average filter for 200 Hz signals in spectrum
  RingBuffer<double, NUM_OF_AVG_VALUES> avgValues200Channel0; // values for peak detection
  RingBuffer<double, NUM_OF_AVG_VALUES> avgValues200Channel1; // values for peack detection
  
  double correlationChannel0;
  double correlationChannel1;
  int actSamples;
  int cmpCnt;
  
  fftw_complex *freqs1kHz;
  fftw_plan dft1kHz;
  fftw_plan                           fft2048, ifft2048; // plan for FFT and inverse FFT of an array of size 2048

  double *sound1kHz;
  double *whistleInput8kHz;
 // double magSpectrumChannel0[DFT_LENGTH / 2 + 1]; // magnitude spectrum of the fft
 // double magSpectrumChannel1[DFT_LENGTH / 2 + 1]; // magnitude spectrum of the fft
  double avgValue320Channel0;  // average value over AVG_FILTER_LEN values of FFT at 320Hz
  double avgValue320Channel1;  // average value over AVG_FILTER_LEN values of FFT at 320Hz
  double avgValue200Channel0;  // average value over AVG_FILTER_LEN values of FFT at 200Hz
  double avgValue200Channel1;  // average value over AVG_FILTER_LEN values of FFT at 200Hz
  double                              *corrBuff;         // Buffer of the resulting correlation
  fftw_complex                        *fftDataIn;        // Buffer of the FFT of data in
  std::vector<int> peakBuffer;

  
  /**
  * The method that detects the signals
  *
  * @param SoundSignal The identified signal
  */
  void update(SoundSignal& soundSignal);
  
  void downsamplingAndLowpassFiltering();
  
  void detectAFSKSignal(const RingBuffer<float, LOWPASS_BUFFER_LENGTH>& lowPassOutput,
                        RingBuffer<double, AVG_FILTER_LEN>& avgFilter, int freqIndex,
                        double& avgValue);
  
  bool detectWhistle(const RingBuffer<float, WHISTLE_BUFF_LEN>& inputChannel, double& correlation);
  
  int getPeaks(const RingBuffer<double, NUM_OF_AVG_VALUES>& values);
  
  void plot();
  
public:
  /** Constructor */
  SoundSignalRecognizer();
  
  /** Destructor */
  ~SoundSignalRecognizer();
};

#endif
