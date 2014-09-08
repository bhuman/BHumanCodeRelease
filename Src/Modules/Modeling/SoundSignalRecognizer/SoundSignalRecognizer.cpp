/*
 * @file SoundSignalRecognizer.cpp
 *
 * Implementation of module that identifies audio signals for the
 * Sound Recognition Challenge 2014
 *
 * @author Dennis Schuethe
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#ifndef WINDOWS

#include "SoundSignalRecognizer.h"
#include <limits>
#include "Data.h"

MAKE_MODULE(SoundSignalRecognizer, Modeling)


SoundSignalRecognizer::SoundSignalRecognizer()
{
  for(int i = 0; i < WHISTLE_BUFF_LEN; ++i)
  {
    inputChannel0.add(0);
    inputChannel1.add(0);
  }
  for(int i = 0; i < LOWPASS_BUFFER_LENGTH; ++i)
  {
    lowPassOutput0.add(0);
    lowPassOutput1.add(0);
  }
  for(int i = 0; i < AVG_FILTER_LEN; ++i)
  {
    avgFilter320Channel0.add(0);
    avgFilter320Channel1.add(0);
    avgFilter200Channel0.add(0);
    avgFilter200Channel1.add(0);
  }
  actSamples = 0;
  avgValue320Channel0 = 0.f;
  avgValue320Channel1 = 0.f;
  avgValue200Channel0 = 0.f;
  avgValue200Channel1 = 0.f;
  cmpCnt = 0;
  correlationChannel0 = 0.0;
  correlationChannel1 = 0.0;
  
  // First get a plan for in and out buffer and allocate sizes
  //sound1kHz = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  sound1kHz = (double*) fftw_malloc(sizeof(double) * DFT_LENGTH);
  whistleInput8kHz = (double*) fftw_malloc(sizeof(double) * WHISTLE_FFT_LEN);
  freqs1kHz = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DFT_LENGTH/2 + 1));
  fftDataIn = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (WHISTLE_BUFF_LEN + 1));
  corrBuff  = (double*) fftw_malloc(sizeof(double) * WHISTLE_FFT_LEN);

  
  // create plan - NOTE: after plan has been created, in and out buffer are set to zero
  // plan has to be created only once!!!
  dft1kHz  = fftw_plan_dft_r2c_1d(DFT_LENGTH, sound1kHz, freqs1kHz, FFTW_MEASURE); // build plan that fftw needs to compute the fft
  fft2048  = fftw_plan_dft_r2c_1d(WHISTLE_FFT_LEN, whistleInput8kHz, fftDataIn, FFTW_MEASURE);   // build plan that fftw needs to compute the fft
  ifft2048 = fftw_plan_dft_c2r_1d(WHISTLE_FFT_LEN, fftDataIn, corrBuff, FFTW_MEASURE); // build plan that fftw needs to compute the fft

}

SoundSignalRecognizer::~SoundSignalRecognizer()
{
  fftw_destroy_plan(dft1kHz);
  fftw_free(sound1kHz);
  fftw_free(freqs1kHz);
}

void SoundSignalRecognizer::update(SoundSignal& soundSignal)
{
  // Check input data:
  if(theAudioData.channels != 2)
  {
    OUTPUT(idText, text, "Wrong number of channels! SoundSignalRecognizer expects 2 channels, but AudioData has " << theAudioData.channels << "!");
  }
  if(theAudioData.samples.size() == 0)
    return;
  else
    soundSignal.lastTimeOfIncomingSound = theFrameInfo.time;
  
  // Step 1: low-pass filtering of input data and downsampling from 8 kHz to 1 kHz
  downsamplingAndLowpassFiltering();
  
  // Step 2:
  /*
   * check if the filter output is full and the overlap flag is reached, which
   * means that OVERLAP samples are new inputs for the dft and FILTER_OUT_LENGTH-OVERLAP
   * old samples remain in the buffer
   */
  if(lowPassOutput0.isFilled() && actSamples >= OVERLAP-1 && !disableSignalRecognition)
  {
    actSamples = 0;
    // Search for 320 Hz signal:
    detectAFSKSignal(lowPassOutput0, avgFilter320Channel0, F320, avgValue320Channel0);
    avgValues320Channel0.add(avgFilter320Channel0.getEntry(0));
    detectAFSKSignal(lowPassOutput1, avgFilter320Channel1, F320, avgValue320Channel1);
    avgValues320Channel1.add(avgFilter320Channel1.getEntry(0));
    // Search for 200 Hz signal
    detectAFSKSignal(lowPassOutput0, avgFilter200Channel0, F200, avgValue200Channel0);
    avgValues200Channel0.add(avgFilter200Channel0.getEntry(0));
    detectAFSKSignal(lowPassOutput1, avgFilter200Channel1, F200, avgValue200Channel1);
    avgValues200Channel1.add(avgFilter200Channel1.getEntry(0));

    const int peaks320Channel0 = getPeaks(avgValues320Channel0);
    const int peaks320Channel1 = getPeaks(avgValues320Channel1);
    const int peaks200Channel0 = getPeaks(avgValues200Channel0);
    const int peaks200Channel1 = getPeaks(avgValues200Channel1);
    if(peaks320Channel0 >= minNumOfPeaks && peaks320Channel1 >= minNumOfPeaks &&
       peaks200Channel0 >= minNumOfPeaks && peaks200Channel1 >= minNumOfPeaks &&
       peaks320Channel0 <= maxNumOfPeaks && peaks320Channel1 <= maxNumOfPeaks &&
       peaks200Channel0 <= maxNumOfPeaks && peaks200Channel1 <= maxNumOfPeaks)
    {
      soundSignal.lastTimeAFSKSignalDetected = theFrameInfo.time;
      OUTPUT(idText, text, "I heard the sound!");
    }
  }
  
  // Step 3: The Whistle
  if (inputChannel0.isFilled() && cmpCnt>=WHISTLE_OVERLAP && !disableWhistleRecognition)
  {
    cmpCnt = 0;
    const bool w0 = detectWhistle(inputChannel0, correlationChannel0);
    const bool w1 = detectWhistle(inputChannel1, correlationChannel1);
    if((whistleNeedsToBeInBothChannels && w0 && w1) || (!whistleNeedsToBeInBothChannels && (w0 || w1)))
      soundSignal.lastTimeWhistleDetected = theFrameInfo.time;
  }
  
  // Finally, plot some stuff
  if(!deactivatePlots)
    plot();
}

void SoundSignalRecognizer::downsamplingAndLowpassFiltering()
{
  unsigned int i = 0;
  while(i < theAudioData.samples.size())
  {
    cmpCnt++;
    const float sample0 = static_cast<float>(theAudioData.samples[i]) / static_cast<float>(std::numeric_limits<short>::max());
    ++i;
    const float sample1 = static_cast<float>(theAudioData.samples[i]) / static_cast<float>(std::numeric_limits<short>::max());
    ++i;
    inputChannel0.add(sample0);
    inputChannel1.add(sample1);
    if(i % 16 == 0)
    {
      actSamples += 1;
      float actValue0 = 0.f;
      float actValue1 = 0.f;
      for(int j = 0; j < INPUT_BUFFER_LENGTH; j++)
      {
        actValue0 += inputChannel0.getEntry(j) * fc[j];
        actValue1 += inputChannel1.getEntry(j) * fc[j];
      }
      lowPassOutput0.add(actValue0);
      lowPassOutput1.add(actValue1);
    }
  }
}

void SoundSignalRecognizer::detectAFSKSignal(const RingBuffer<float, LOWPASS_BUFFER_LENGTH>& lowPassOutput,
                                             RingBuffer<double, AVG_FILTER_LEN>& avgFilter, int freqIndex,
                                             double& avgValue)
{
  for (int i=0;i<LOWPASS_BUFFER_LENGTH;i++)
  {
    sound1kHz[i] = lowPassOutput.getEntry(i);
  }
  fftw_execute(dft1kHz); /*	compute the fft using arrays in and out - repeat as needed */
  
  // for testing only ---- can be removed later
  //for (int i=0; i<DFT_LENGTH/2+1; i++)
  //{
  //  magSpectrumChannel0[i] = std::sqrt(freqs1kHz[i][REAL]*freqs1kHz[i][REAL]+freqs1kHz[i][IMAG]*freqs1kHz[i][IMAG]);
  //}
  
  /*
   * calculate here the average value over AVG_FILTER_LEN values of the FFT
   * at 320Hz signal. Same can be done for 320Hz, iff needed
   */
  const double freqAbs = std::sqrt(freqs1kHz[freqIndex][REAL]*freqs1kHz[freqIndex][REAL]+
                                   freqs1kHz[freqIndex][IMAG]*freqs1kHz[freqIndex][IMAG]);
  avgValue = (freqAbs - avgFilter.getEntry(AVG_FILTER_LEN-1)) / AVG_FILTER_LEN + avgValue;
  avgFilter.add(freqAbs);
  
  /*
   * TODO
   * Check here if avgValue is above threshold or store M values of avgValue and the check for maxima which
   * have nearly same magnitude and are k points away from each other (which is the time T between
   * two signals of same signal source multiplied by the sampling frequency fs --> (int)T*fs
   */
}

bool SoundSignalRecognizer::detectWhistle(const RingBuffer<float, WHISTLE_BUFF_LEN>& inputChannel, double& correlation)
{
  for (int j=0; j<WHISTLE_BUFF_LEN; j++)
  {
    whistleInput8kHz[j] = inputChannel.getEntry(j); // write just the half of dataIn and the rest is zero padded
  }
  fftw_execute(fft2048); /*	compute the fft - repeat as needed */
  /*
   * Now multiply the FFT of dataIn with the conjugete FFT of cmpData
   */
  for (int j=0; j<WHISTLE_BUFF_LEN+1; j++)
  {
    const double realFFTDataIn = fftDataIn[j][REAL];
    fftDataIn[j][REAL] = realFFTDataIn*fftCmpData[j][REAL] - fftDataIn[j][IMAG]*fftCmpData[j][IMAG]; // real x real - imag x imag
    fftDataIn[j][IMAG] = fftDataIn[j][IMAG]*fftCmpData[j][REAL] + realFFTDataIn*fftCmpData[j][IMAG]; // real x imag + imag x real
  }
  fftw_execute(ifft2048); // calculate the ifft which is now the same as the correlation
  
  /*
   * Post Processing:
   * Find the maximum of the correlation and before weight by the
   * highest value possible, which is given by the auto correlation
   */
  correlation = 0.0;
  for (int j=0; j<WHISTLE_CORR_LEN; j++)
  {
    corrBuff[j] /= MAX_AC_VALUE;
    if(correlation < corrBuff[j])
      correlation = corrBuff[j];
    else if(correlation < -corrBuff[j])
      correlation = -corrBuff[j];
  }
  if (correlation >= whistleThreshold)
  {
    OUTPUT(idText, text, "Whistle has been blown!" << correlation);
    return true;
  }
  return false;
}

int SoundSignalRecognizer::getPeaks(const RingBuffer<double, NUM_OF_AVG_VALUES>& values)
{
  if(!values.isFilled())
    return 0;
  double avgVal = 0.0;
  for(int i=0; i<values.getNumberOfEntries(); i++)
  {
    avgVal += values.getEntry(i);
  }
  avgVal /= values.getNumberOfEntries();
  peakBuffer.clear();
  for(int i=3; i<values.getNumberOfEntries()-3; i++)
  {
    const double val = values.getEntry(i);
    if(val < afskSignalStrengthThreshold || val < avgVal)
      continue;
    bool isPeak = true;
    for(int j=i-3; j<=i+3; j++)
    {
      if(values.getEntry(j) > val)
      {
        isPeak = false;
        break;
      }
    }
    if(isPeak)
    {
      peakBuffer.push_back(i);
    }
  }
  if(peakBuffer.size() > 1)
  {
    bool peaksAreOK = true;
    // Check, if peaks have the correct distances:
    for(unsigned int i=0; i<peakBuffer.size()-1; ++i)
    {
      const int diff = peakBuffer[i+1] - peakBuffer[i];
      if(diff < minPeakDistanceInBuffer || diff > maxPeakDistanceInBuffer)
        peaksAreOK = false;
    }
    if(peaksAreOK)
      return static_cast<int>(peakBuffer.size());
    else
      return 0;
  }
  return static_cast<int>(peakBuffer.size());
}

void SoundSignalRecognizer::plot()
{
  // AFSK signal detection buffer:
  DECLARE_PLOT("module:SoundSignalRecognizer:avgValue320Channel0");
  DECLARE_PLOT("module:SoundSignalRecognizer:avgValue320Channel1");
  PLOT("module:SoundSignalRecognizer:avgValue320Channel0", avgValues320Channel0.getEntry(0));
  PLOT("module:SoundSignalRecognizer:avgValue320Channel1", avgValues320Channel1.getEntry(0));
  PLOT("module:SoundSignalRecognizer:lastValue320Channel0", avgFilter320Channel0.getEntry(0));
  PLOT("module:SoundSignalRecognizer:lastValue320Channel1", avgFilter320Channel1.getEntry(0));
  DECLARE_PLOT("module:SoundSignalRecognizer:avgValue200Channel0");
  DECLARE_PLOT("module:SoundSignalRecognizer:avgValue200Channel1");
  PLOT("module:SoundSignalRecognizer:avgValue200Channel0", avgValues200Channel0.getEntry(0));
  PLOT("module:SoundSignalRecognizer:avgValue200Channel1", avgValues200Channel1.getEntry(0));
  PLOT("module:SoundSignalRecognizer:lastValue200Channel0", avgFilter200Channel0.getEntry(0));
  PLOT("module:SoundSignalRecognizer:lastValue200Channel1", avgFilter200Channel1.getEntry(0));
  
  // Whistle correlation:
  DECLARE_PLOT("module:SoundSignalRecognizer:whistleCorrelationChannel0");
  DECLARE_PLOT("module:SoundSignalRecognizer:whistleCorrelationChannel1");
  PLOT("module:SoundSignalRecognizer:whistleCorrelationChannel0", correlationChannel0);
  PLOT("module:SoundSignalRecognizer:whistleCorrelationChannel1", correlationChannel1);
  
  // DFT
  DECLARE_PLOT("module:SoundSignalRecognizer:DFT");
  for(int i=0; i<(DFT_LENGTH / 2) + 1; ++i)
  {
    const double freqAbs = std::sqrt(freqs1kHz[i][REAL]*freqs1kHz[i][REAL]+
                                     freqs1kHz[i][IMAG]*freqs1kHz[i][IMAG]);
    PLOT("module:SoundSignalRecognizer:DFT", freqAbs);
  }
}

#endif
