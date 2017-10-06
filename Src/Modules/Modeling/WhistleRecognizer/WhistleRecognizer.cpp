/*
 * @file WhistleRecognizer.cpp
 *
 * Implementation of module that identifies the sound of a whistle.
 * It is possible to compare the current sound to multiple reference whistles.
 *
 * @author Dennis Schuethe
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "WhistleRecognizer.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(WhistleRecognizer, modeling)

// We currently do not have a working FFTW3 implementation
// for Windows in our repository.

#include <limits>
#include "Platform/Thread.h"

static DECLARE_SYNC;

WhistleRecognizer::WhistleRecognizer()
{
  for(int i = 0; i < WHISTLE_BUFF_LEN; ++i)
  {
    inputChannel0.push_front(0);
    inputChannel1.push_front(0);
  }
  numberOfNewAudioSamples = 0;
  lastTimeWhistleDetectedInBothChannels = 0;
  lastTimefft2048Computed = 0;

  // Allocate memeory for FFTW plans
  whistleInput8kHz = (double*) fftw_malloc(sizeof(double) * WHISTLE_FFT_LEN);
  fftDataIn        = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (WHISTLE_BUFF_LEN + 1));
  corrBuff         = (double*) fftw_malloc(sizeof(double) * WHISTLE_FFT_LEN);

  // Create plans
  //  - after plan has been created, in and out buffer are set to zero
  //  - plan has to be created only once
  // Creation of FFTW plans is not thread-safe, thus we need to synchronize with the other threads
  //   - This is only relevant for simulations that contain multiple robots
  SYNC;
  fft2048  = fftw_plan_dft_r2c_1d(WHISTLE_FFT_LEN, whistleInput8kHz, fftDataIn, FFTW_MEASURE);   // build plan that fftw needs to compute the fft
  ifft2048 = fftw_plan_dft_c2r_1d(WHISTLE_FFT_LEN, fftDataIn, corrBuff, FFTW_MEASURE); // build plan that fftw needs to compute the fft

  // Load reference whistle
  ASSERT(whistleFiles.size());
  for(unsigned int i = 0; i < whistleFiles.size(); ++i)
  {
    fftw_complex* loadCmpData = new fftw_complex[WHISTLE_BUFF_LEN + 1];
    float maxCorrelation;
    loadReferenceWhistle(loadCmpData, maxCorrelation, whistleFiles[i]);
    maxAutoCorrelationValue.push_back(maxCorrelation);
    fftCmpData.push_back(loadCmpData);
  }
  newfftCmpData = new fftw_complex[WHISTLE_BUFF_LEN + 1];
}

WhistleRecognizer::~WhistleRecognizer()
{
  // Destruction of FFTW plans is not thread-safe
  SYNC;
  fftw_destroy_plan(fft2048);
  fftw_destroy_plan(ifft2048);
  fftw_free(whistleInput8kHz);
  fftw_free(fftDataIn);
  fftw_free(corrBuff);
  for(unsigned int i = 0; i < fftCmpData.size(); ++i)
    delete [](fftCmpData[i]);
  delete [] newfftCmpData;
}

void WhistleRecognizer::update(Whistle& whistle)
{
  DEBUG_RESPONSE_ONCE("module:WhistleRecognizer:whistle")
  {
    whistle.lastTimeWhistleDetected = theFrameInfo.time;
    whistle.lastTimeOfIncomingSound = theFrameInfo.time;
    whistle.confidenceOfLastWhistleDetection = 100;
    return;
  }
  DEBUG_RESPONSE_ONCE("module:WhistleRecognizer:sound")
  {
    whistle.lastTimeOfIncomingSound = theFrameInfo.time;
    return;
  }

  if(SystemCall::soundIsPlaying())
    lastTimeWithReplayEvent = theFrameInfo.time;

  // Only listen to the whistle in set state and clear
  // the buffers when entering a set state:
  if(theCognitionStateChanges.lastGameState != STATE_SET && theGameInfo.state == STATE_SET)
  {
    for(int i = 0; i < WHISTLE_BUFF_LEN; ++i)
    {
      inputChannel0.push_front(0);
      inputChannel1.push_front(0);
    }
    numberOfNewAudioSamples = 0;
  }
  if(theGameInfo.state != STATE_SET)
    return;

  // Check input data:
  if(theAudioData.channels != 2)
  {
    OUTPUT_TEXT("Wrong number of channels! WhistleRecognizer expects 2 channels, but AudioData has " << theAudioData.channels << "!");
  }
  if(theAudioData.samples.size() == 0)
    return;
  else
    whistle.lastTimeOfIncomingSound = theFrameInfo.time;

  // Add incoming audio data to the two buffers
  unsigned int i = 0;
  while(i < theAudioData.samples.size())
  {
    numberOfNewAudioSamples++;
    const float sample0 = static_cast<float>(theAudioData.samples[i]) / static_cast<float>(std::numeric_limits<short>::max());
    ++i;
    const float sample1 = static_cast<float>(theAudioData.samples[i]) / static_cast<float>(std::numeric_limits<short>::max());
    ++i;
    inputChannel0.push_front(sample0);
    inputChannel1.push_front(sample1);
  }

  if(theFrameInfo.getTimeSince(lastTimeWithReplayEvent) < soundReplayDelay)
  {
    if(whistle.confidenceOfLastWhistleDetection >= 0)
      whistle.confidenceOfLastWhistleDetection = -whistle.confidenceOfLastWhistleDetection;
    return;
  }
  else if(whistle.confidenceOfLastWhistleDetection < -1)
    whistle.confidenceOfLastWhistleDetection = -whistle.confidenceOfLastWhistleDetection;

  DEBUG_RESPONSE_ONCE("module:WhistleRecognizer:saveReferenceWhistle")
  {
    saveReferenceWhistle();
  }
  DEBUG_RESPONSE_ONCE("module:WhistleRecognizer:recordNewReferenceWhistle")
  {
    if(inputChannel0.full())
    {
      OUTPUT_TEXT("Record new Whistle");
      recordNewReferenceWhistle();
    }
  }

  // Recognize the whistle
  float bestCorrelationChannel0 = 0.0;
  float bestCorrelationChannel1 = 0.0;
  float  currentVolume = 0.0;
  int bestWhistleIndex0 = -1;
  int bestWhistleIndex1 = -1;
  if(inputChannel0.full() && numberOfNewAudioSamples >= WHISTLE_OVERLAP)
  {
    numberOfNewAudioSamples = 0;
    currentVolume  = computeCurrentVolume();
    const bool vol = currentVolume > volumeThreshold;

    std::vector<float> correlationsC0;
    std::vector<float> correlationsC1;
    for(unsigned int i = 0; i < whistleFiles.size(); ++i)
    {
      float c0Corr = getWhistleCorrelationInPercent(inputChannel0, i);
      float c1Corr = getWhistleCorrelationInPercent(inputChannel1, i);
      correlationsC0.push_back(c0Corr);
      correlationsC1.push_back(c1Corr);
    }
    bestWhistleIndex0 = listContainsValueHigherThan100(correlationsC0, bestCorrelationChannel0);
    bestWhistleIndex1 = listContainsValueHigherThan100(correlationsC1, bestCorrelationChannel1);
    bool w0 = bestWhistleIndex0 != -1;
    bool w1 = bestWhistleIndex1 != -1;
    if(w0)
      printCorrelationResults(0, correlationsC0);
    if(w1)
      printCorrelationResults(1, correlationsC1);
    int brokenFirst = (theDamageConfigurationHead.audioChannelsDefect[0] ? 1 : 0) + (theDamageConfigurationHead.audioChannelsDefect[1] ? 1 : 0);
    int brokenSecond = (theDamageConfigurationHead.audioChannelsDefect[2] ? 1 : 0) + (theDamageConfigurationHead.audioChannelsDefect[3] ? 1 : 0);
    int firstChannel = brokenFirst > brokenSecond ? 2 : 0;
    audioChannel0Defect = theDamageConfigurationHead.audioChannelsDefect[firstChannel];
    audioChannel1Defect = theDamageConfigurationHead.audioChannelsDefect[firstChannel + 1];

    // Best case: Everything is fine!
    if(w0 && w1 && vol && !audioChannel0Defect && !audioChannel1Defect)
    {
      lastTimeWhistleDetectedInBothChannels = theFrameInfo.time;
      currentWhistle.lastTimeWhistleDetected = theFrameInfo.time;
      currentWhistle.confidenceOfLastWhistleDetection = 100;
    }
    // One ear ist damaged but I can hear the sound on the other ear:
    else if((w0 && vol && !audioChannel0Defect && audioChannel1Defect) ||
            (w1 && vol && !audioChannel1Defect && audioChannel0Defect))
    {
      currentWhistle.lastTimeWhistleDetected = theFrameInfo.time;
      currentWhistle.confidenceOfLastWhistleDetection = 66;
    }
    // Last (positive) case: Both ears are OK, but I can hear the sound on one ear, only:
    else if(!audioChannel0Defect && !audioChannel1Defect &&
            ((w0 && !w1) || (!w0 && w1)) && vol)
    {
      currentWhistle.lastTimeWhistleDetected = theFrameInfo.time;
      if(theFrameInfo.getTimeSince(lastTimeWhistleDetectedInBothChannels) > timeForOneChannelAcceptance)
        currentWhistle.confidenceOfLastWhistleDetection = 33;
      else
        currentWhistle.confidenceOfLastWhistleDetection = 100;
    }
    // Finally, a completely deaf robot has a negative confidence:
    else if(audioChannel0Defect && audioChannel1Defect)
    {
      whistle.confidenceOfLastWhistleDetection = -1; // Not a detection but other robots get the information to ignore me
    }
  }

  if(currentWhistle.lastTimeWhistleDetected == theFrameInfo.time)
    lastDetectedWhistles.push(currentWhistle);

  while(!lastDetectedWhistles.empty() && theFrameInfo.getTimeSince(lastDetectedWhistles.front().lastTimeWhistleDetected) > whistleCombinationPeriod)
    lastDetectedWhistles.pop();

  if(lastDetectedWhistles.size() >= minConsecutiveWhistles)
  {
    whistle.confidenceOfLastWhistleDetection = lastDetectedWhistles.back().confidenceOfLastWhistleDetection;
    whistle.lastTimeWhistleDetected = lastDetectedWhistles.back().lastTimeWhistleDetected;
  }

  if(whistle.lastTimeWhistleDetected == theFrameInfo.time)
  {
    whistle.whistleName = (bestWhistleIndex0 != -1 ? "(" + whistleFiles[bestWhistleIndex0] + ")" : "()") + "/" +
                          (bestWhistleIndex1 != -1 ? "(" + whistleFiles[bestWhistleIndex1] + ")" : "()");
    ANNOTATION("WhistleRecognizer", "I heard the " << whistle.whistleName << " whistle :-)  C0: " << bestCorrelationChannel0 << "  C1: " << bestCorrelationChannel1);
  }

  // Finally, plot the whistle correlation of each channel:
  if(!deactivatePlots)
  {
    DECLARE_PLOT("module:WhistleRecognizer:whistleCorrelationChannel0");
    DECLARE_PLOT("module:WhistleRecognizer:whistleCorrelationChannel1");
    PLOT("module:WhistleRecognizer:whistleCorrelationChannel0", bestCorrelationChannel0);
    PLOT("module:WhistleRecognizer:whistleCorrelationChannel1", bestCorrelationChannel1);
    DECLARE_PLOT("module:WhistleRecognizer:audioInput60HzChannel0");
    DECLARE_PLOT("module:WhistleRecognizer:audioInput60HzChannel1");
    PLOT("module:WhistleRecognizer:audioInput60HzChannel0", inputChannel0.back());
    PLOT("module:WhistleRecognizer:audioInput60HzChannel1", inputChannel1.back());
    DECLARE_PLOT("module:WhistleRecognizer:currentVolume");
    PLOT("module:WhistleRecognizer:currentVolume", currentVolume);
  }
}

float WhistleRecognizer::getWhistleCorrelationInPercent(const RingBuffer<float, WHISTLE_BUFF_LEN>& inputChannel, unsigned whistleNumber)
{
  //  if(lastTimefft2048Computed != theFrameInfo.time)
  //  {
  for(int j = 0; j < WHISTLE_BUFF_LEN; j++)
  {
    whistleInput8kHz[j] = inputChannel[j]; // write just the half of dataIn and the rest is zero padded
  }
  fftw_execute(fft2048); /* compute the fft - repeat as needed */
  //    lastTimefft2048Computed = theFrameInfo.time;
  //  }
  /*
   * Now multiply the FFT of dataIn with the conjugete FFT of cmpData
   */
  const int REAL = 0;     // real value of complex
  const int IMAG = 1;     // imaginary value of complex
  for(int j = 0; j < WHISTLE_BUFF_LEN + 1; j++)
  {
    const double realFFTDataIn = fftDataIn[j][REAL];
    fftDataIn[j][REAL] = realFFTDataIn * fftCmpData[whistleNumber][j][REAL] - fftDataIn[j][IMAG] * fftCmpData[whistleNumber][j][IMAG]; // real x real - imag x imag
    fftDataIn[j][IMAG] = fftDataIn[j][IMAG] * fftCmpData[whistleNumber][j][REAL] + realFFTDataIn * fftCmpData[whistleNumber][j][IMAG]; // real x imag + imag x real
  }
  fftw_execute(ifft2048); // calculate the ifft which is now the same as the correlation

  /*
   * Post Processing:
   * Find the maximum of the correlation and before weight by the
   * highest value possible, which is given by the auto correlation
   */
  double correlation = 0.0;
  for(int j = 0; j < WHISTLE_CORR_LEN; j++)
  {
    corrBuff[j] /= maxAutoCorrelationValue[whistleNumber];
    if(correlation < corrBuff[j])
      correlation = corrBuff[j];
    else if(correlation < -corrBuff[j])
      correlation = -corrBuff[j];
  }
  return (static_cast<float>(correlation) / whistleThresholds[whistleNumber]) * 100.f;
}

int WhistleRecognizer::listContainsValueHigherThan100(const std::vector<float>& correlationList, float& bestCorrelation)
{
  ASSERT(correlationList.size() != 0);
  bestCorrelation = correlationList[0];
  int bestIndex = 0;
  for(unsigned int i = 1; i < correlationList.size(); ++i)
  {
    if(correlationList[i] > bestCorrelation)
    {
      bestCorrelation = correlationList[i];
      bestIndex = static_cast<int>(i);
    }
  }
  return bestCorrelation < 100.f ? -1 : bestIndex;
}

void WhistleRecognizer::printCorrelationResults(int channelNumber, const std::vector<float>& correlationList)
{
  OUTPUT_TEXT("I found a correlation in channel " << channelNumber << "!   (YEEEEHAH!) *******************");
  for(unsigned int i = 0; i < correlationList.size(); ++i)
  {
    OUTPUT_TEXT("   " << whistleFiles[i] << ":    " << correlationList[i]);
  }
  OUTPUT_TEXT("*********************************************************************");
}

void WhistleRecognizer::recordNewReferenceWhistle()
{
  for(int j = 0; j < WHISTLE_BUFF_LEN; j++)
  {
    whistleInput8kHz[j] = inputChannel0[j]; // write just the half of dataIn and the rest is zero padded
  }
  fftw_execute(fft2048); /* compute the fft - repeat as needed */
  /*
   * Now multiply the FFT of dataIn with the conjugate FFT of cmpData
   */
  const int REAL = 0;     // real value of complex
  const int IMAG = 1;     // imaginary value of complex
  for(int j = 0; j < WHISTLE_BUFF_LEN + 1; j++)
  {
    newfftCmpData[j][REAL] = fftDataIn[j][REAL];
    newfftCmpData[j][IMAG] = -fftDataIn[j][IMAG];
    const double realFFTDataIn = fftDataIn[j][REAL];
    fftDataIn[j][REAL] = realFFTDataIn * realFFTDataIn + fftDataIn[j][IMAG] * fftDataIn[j][IMAG]; // real x real + imag x imag
    fftDataIn[j][IMAG] = 0.0;
  }
  fftw_execute(ifft2048); // calculate the ifft which is now the same as the correlation

  /*
   * Post Processing:
   * Find the maximum of the correlation and before weight by the
   * highest value possible, which is given by the auto correlation
   */
  double max_cc_value = 0;
  for(int j = 0; j < WHISTLE_CORR_LEN; j++)
  {
    if(max_cc_value < corrBuff[j])
    {
      max_cc_value = corrBuff[j];
    }
    else if(max_cc_value < -corrBuff[j])
    {
      max_cc_value = -corrBuff[j];
    }
  }
  newMaxAutoCorrelationValue = (float)max_cc_value; // this is the new max value which is always the maximum, because of the auto correlation
  OUTPUT_TEXT("A new whistle has been recorded (maxAutoCorrelationValue = " << newMaxAutoCorrelationValue
              << "). To save it to the file " << nameForNewWhistles << ".dat, you have to enter: ");
  OUTPUT_TEXT("dr module:WhistleRecognizer:saveReferenceWhistle");
}

float WhistleRecognizer::computeCurrentVolume()
{
  float volume0 = 0.0;
  float volume1 = 0.0;
  if(!audioChannel0Defect)
  {
    for(float sample : inputChannel0)
      volume0 += std::abs(sample);
  }
  volume0 /= static_cast<float>(inputChannel0.size());
  if(!audioChannel1Defect)
  {
    for(float sample : inputChannel1)
      volume1 += std::abs(sample);
  }
  volume1 /= static_cast<float>(inputChannel1.size());
  if(!audioChannel0Defect && !audioChannel1Defect)
    return (volume0 + volume1) / 2.f;
  else if(audioChannel0Defect)
    return volume1;
  else // this means if(audioChannel1Defect)
    return volume0;
}

void WhistleRecognizer::saveReferenceWhistle()
{
  OutBinaryFile file(std::string("Whistles/") + nameForNewWhistles + ".dat");
  for(int i = 0; i < WHISTLE_BUFF_LEN + 1; i++)
  {
    file << newfftCmpData[i][0];
    file << newfftCmpData[i][1];
  }
  file << newMaxAutoCorrelationValue;
  OUTPUT_TEXT("New Whistle has been saved to " << nameForNewWhistles << ".dat.");
}

void WhistleRecognizer::loadReferenceWhistle(fftw_complex* loadCmpData, float& maxAutoCorrelationValue, const std::string& fileName)
{
  ASSERT(whistleFiles.size());
  InBinaryFile file(std::string("Whistles/") + fileName + ".dat");
  ASSERT(file.exists());
  for(int i = 0; i < WHISTLE_BUFF_LEN + 1; i++)
  {
    file >> loadCmpData[i][0];
    file >> loadCmpData[i][1];
  }
  file >> maxAutoCorrelationValue;
}
