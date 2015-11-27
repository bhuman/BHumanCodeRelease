/*
 * @file WhistleRecognizer.cpp
 *
 * Implementation of module that identifies the sound of a whistle
 *
 * @author Dennis Schuethe
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "WhistleRecognizer.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(WhistleRecognizer, modeling)

#ifndef WINDOWS
// We currently do not have a working FFTW3 implementation
// for Windows in our repository.

#include <limits>
#include "DefaultWhistle.h"
#include "Platform/Thread.h"

static SyncObject _syncObject;

WhistleRecognizer::WhistleRecognizer()
{
  for(int i = 0; i < WHISTLE_BUFF_LEN; ++i)
  {
    inputChannel0.push_front(0);
    inputChannel1.push_front(0);
  }
  maxAutoCorrelationValue = 1522544.77f; // Default value for preconfigured whistle
  cmpCnt = 0;
  lastGameState = STATE_INITIAL;
  lastTimeWhistleDetectedInBothChannels = 0;

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

  // Load reference whistle (memory has already been allocated)
  loadReferenceWhistle();
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
}

#endif

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

#ifdef WINDOWS // Windows implementation ends here, remaning stuff uses FFTW
}
#else

  // Only listen to the whistle in set state and clear
  // the buffers when entering a set state:
  if(lastGameState != STATE_SET && theGameInfo.state == STATE_SET)
  {
    for(int i = 0; i < WHISTLE_BUFF_LEN; ++i)
    {
      inputChannel0.push_front(0);
      inputChannel1.push_front(0);
    }
    cmpCnt = 0;
  }
  lastGameState = theGameInfo.state;
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
    cmpCnt++;
    const float sample0 = static_cast<float>(theAudioData.samples[i]) / static_cast<float>(std::numeric_limits<short>::max());
    ++i;
    const float sample1 = static_cast<float>(theAudioData.samples[i]) / static_cast<float>(std::numeric_limits<short>::max());
    ++i;
    inputChannel0.push_front(sample0);
    inputChannel1.push_front(sample1);
  }

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
  double correlationChannel0 = 0.0;
  double correlationChannel1 = 0.0;
  float  currentVolume = 0.0;
  if(inputChannel0.full() && cmpCnt >= WHISTLE_OVERLAP)
  {
    cmpCnt = 0;
    const bool w0  = detectWhistle(inputChannel0, correlationChannel0);
    const bool w1  = detectWhistle(inputChannel1, correlationChannel1);
    currentVolume  = computeCurrentVolume();
    const bool vol = currentVolume > volumeThreshold;
    // Best case: Everything is fine!
    if(w0 && w1 && vol && !theDamageConfigurationHead.audioChannel0Defect && !theDamageConfigurationHead.audioChannel1Defect)
    {
      lastTimeWhistleDetectedInBothChannels = theFrameInfo.time;
      whistle.lastTimeWhistleDetected = theFrameInfo.time;
      whistle.confidenceOfLastWhistleDetection = 100;
    }
    // One ear ist damaged but I can hear the sound on the other ear:
    else if((w0 && vol && !theDamageConfigurationHead.audioChannel0Defect && theDamageConfigurationHead.audioChannel1Defect) ||
            (w1 && vol && !theDamageConfigurationHead.audioChannel1Defect && theDamageConfigurationHead.audioChannel0Defect))
    {
      whistle.lastTimeWhistleDetected = theFrameInfo.time;
      whistle.confidenceOfLastWhistleDetection = 66;
    }
    // Last (positive) case: Both ears are OK, but I can hear the sound on one ear, only:
    else if(!theDamageConfigurationHead.audioChannel0Defect && !theDamageConfigurationHead.audioChannel1Defect &&
            ((w0 && !w1) || (!w0 && w1)) && vol)
    {
      whistle.lastTimeWhistleDetected = theFrameInfo.time;
      if(theFrameInfo.getTimeSince(lastTimeWhistleDetectedInBothChannels) > timeForOneChannelAcceptance)
        whistle.confidenceOfLastWhistleDetection = 33;
      else
        whistle.confidenceOfLastWhistleDetection = 100;
    }
    // Finally, a completely deaf robot has a negative confidence:
    else if(theDamageConfigurationHead.audioChannel0Defect && theDamageConfigurationHead.audioChannel1Defect)
    {
      whistle.confidenceOfLastWhistleDetection = -1; // Not a detection but other robots get the information to ignore me
    }
  }

  // Finally, plot the whistle correlation of each channel:
  if(!deactivatePlots)
  {
    DECLARE_PLOT("module:WhistleRecognizer:whistleCorrelationChannel0");
    DECLARE_PLOT("module:WhistleRecognizer:whistleCorrelationChannel1");
    PLOT("module:WhistleRecognizer:whistleCorrelationChannel0", correlationChannel0);
    PLOT("module:WhistleRecognizer:whistleCorrelationChannel1", correlationChannel1);
    DECLARE_PLOT("module:WhistleRecognizer:audioInput60HzChannel0");
    DECLARE_PLOT("module:WhistleRecognizer:audioInput60HzChannel1");
    PLOT("module:WhistleRecognizer:audioInput60HzChannel0", inputChannel0.back());
    PLOT("module:WhistleRecognizer:audioInput60HzChannel1", inputChannel1.back());
    DECLARE_PLOT("module:WhistleRecognizer:currentVolume");
    PLOT("module:WhistleRecognizer:currentVolume", currentVolume);
  }
}

bool WhistleRecognizer::detectWhistle(const RingBuffer<float, WHISTLE_BUFF_LEN>& inputChannel, double& correlation)
{
  for (int j=0; j<WHISTLE_BUFF_LEN; j++)
  {
    whistleInput8kHz[j] = inputChannel[j]; // write just the half of dataIn and the rest is zero padded
  }
  fftw_execute(fft2048); /* compute the fft - repeat as needed */
  /*
   * Now multiply the FFT of dataIn with the conjugete FFT of cmpData
   */
  const int REAL = 0;     // real value of complex
  const int IMAG = 1;     // imaginary value of complex
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
    corrBuff[j] /= maxAutoCorrelationValue;
    if(correlation < corrBuff[j])
      correlation = corrBuff[j];
    else if(correlation < -corrBuff[j])
      correlation = -corrBuff[j];
  }
  if (correlation >= whistleThreshold)
  {
    OUTPUT_TEXT("Whistle has been blown!" << correlation);
    return true;
  }
  return false;
}

void WhistleRecognizer::recordNewReferenceWhistle()
{
  for (int j=0; j<WHISTLE_BUFF_LEN; j++)
  {
    whistleInput8kHz[j] = inputChannel0[j]; // write just the half of dataIn and the rest is zero padded
  }
  fftw_execute(fft2048); /* compute the fft - repeat as needed */
  /*
   * Now multiply the FFT of dataIn with the conjugate FFT of cmpData
   */
  const int REAL = 0;     // real value of complex
  const int IMAG = 1;     // imaginary value of complex
  for (int j=0; j<WHISTLE_BUFF_LEN+1; j++)
  {
    fftCmpData[j][REAL] = fftDataIn[j][REAL];
    fftCmpData[j][IMAG] = -fftDataIn[j][IMAG];
    const double realFFTDataIn = fftDataIn[j][REAL];
    fftDataIn[j][REAL] = realFFTDataIn*realFFTDataIn + fftDataIn[j][IMAG]*fftDataIn[j][IMAG]; // real x real + imag x imag
    fftDataIn[j][IMAG] = 0.0;
  }
  fftw_execute(ifft2048); // calculate the ifft which is now the same as the correlation

  /*
   * Post Processing:
   * Find the maximum of the correlation and before weight by the
   * highest value possible, which is given by the auto correlation
   */
  double max_cc_value = 0;
  for (int j=0; j<WHISTLE_CORR_LEN; j++) {
    if(max_cc_value < corrBuff[j]) {
      max_cc_value = corrBuff[j];
    } else if(max_cc_value<-corrBuff[j]){
      max_cc_value = -corrBuff[j];
    }
  }
  maxAutoCorrelationValue = (float)max_cc_value; // this is the new max value which is always the maximum, because of the auto correlation
  OUTPUT(idText,text, "New maxAutoCorrelationValue = " << maxAutoCorrelationValue);
}

float WhistleRecognizer::computeCurrentVolume()
{
  float volume0 = 0.0;
  float volume1 = 0.0;
  if(!theDamageConfigurationHead.audioChannel0Defect)
  {
    for(float sample : inputChannel0)
      volume0 += std::abs(sample);
  }
  volume0 /= static_cast<float>(inputChannel0.size());
  if(!theDamageConfigurationHead.audioChannel1Defect)
  {
    for(float sample : inputChannel1)
      volume1 += std::abs(sample);
  }
  volume1 /= static_cast<float>(inputChannel1.size());
  if(!theDamageConfigurationHead.audioChannel0Defect && !theDamageConfigurationHead.audioChannel1Defect)
    return (volume0 + volume1) / 2.f;
  else if(theDamageConfigurationHead.audioChannel0Defect)
    return volume1;
  else // this means if(theDamageConfiguration.audioChannel1Defect)
    return volume0;
}

void WhistleRecognizer::saveReferenceWhistle()
{
  OutBinaryFile file(std::string("Locations/")+ Global::getSettings().location + "/" + whistleFile);
  for(int i=0; i<WHISTLE_BUFF_LEN+1; i++)
  {
    file << fftCmpData[i][0];
    file << fftCmpData[i][1];
  }
  file << maxAutoCorrelationValue;
}

void WhistleRecognizer::loadReferenceWhistle()
{
  InBinaryFile file(whistleFile);
  if(!file.exists())
  {
    OUTPUT_TEXT("No " << whistleFile << " found, using default reference whistle!");
    return;
  }
  for(int i=0; i<WHISTLE_BUFF_LEN+1; i++)
  {
    file >> fftCmpData[i][0];
    file >> fftCmpData[i][1];
  }
  file >> maxAutoCorrelationValue;
}

#endif
