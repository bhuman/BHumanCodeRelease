/*
 * @file NoWirelessSoundRecognizer.cpp
 *
 * Implementation of a module that identifies different sounds for the No Wireless Challenge.
 *
 * @author Dennis Schuethe
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "NoWirelessSoundRecognizer.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(NoWirelessSoundRecognizer, communication)

#ifndef WINDOWS
// We currently do not have a working FFTW3 implementation
// for Windows in our repository.

#include <limits>
#include "Platform/Thread.h"

DECLARE_SYNC_STATIC;

NoWirelessSoundRecognizer::NoWirelessSoundRecognizer()
{
  for(int i = 0; i < SOUND_BUFF_LEN; ++i)
  {
    inputChannel0.push_front(0);
    inputChannel1.push_front(0);
  }
  numberOfNewAudioSamples = 0;
  lastTimeSoundDetectedInBothChannels = 0;

  // Allocate memeory for FFTW plans
  soundInput8kHz   = (double*) fftw_malloc(sizeof(double) * SOUND_FFT_LEN);
  fftDataIn        = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (SOUND_BUFF_LEN + 1));

  // Create plan
  //  - after plan has been created, in and out buffer are set to zero
  //  - plan has to be created only once
  // Creation of FFTW plans is not thread-safe, thus we need to synchronize with the other threads
  //   - This is only relevant for simulations that contain multiple robots
  SYNC;
  fft2048  = fftw_plan_dft_r2c_1d(SOUND_FFT_LEN, soundInput8kHz, fftDataIn, FFTW_MEASURE);   // build plan that fftw needs to compute the fft
}

NoWirelessSoundRecognizer::~NoWirelessSoundRecognizer()
{
  // Destruction of FFTW plans is not thread-safe
  SYNC;
  fftw_destroy_plan(fft2048);
  fftw_free(soundInput8kHz);
  fftw_free(fftDataIn);
}

#endif

void NoWirelessSoundRecognizer::update(NoWirelessSound& noWirelessSound)
{
  noWirelessSound.numOfDifferentSounds = static_cast<int>(sounds.size());
  DECLARE_DEBUG_DRAWING("module:NoWirelessSoundRecognizer:frequencies", "drawingOnField");
  bool outputFrequenciesWhenOverThresh = false;
  MODIFY("module:NoWirelessSoundRecognizer:outputFrequenciesWhenOverThresh", outputFrequenciesWhenOverThresh);
  
#ifdef WINDOWS // Windows implementation ends here, remaning stuff uses FFTW
}
#else

  // Check input data:
  if(theAudioData.channels != 2)
  {
    OUTPUT_TEXT("Wrong number of channels! NoWirelessSoundRecognizer expects 2 channels, but AudioData has " << theAudioData.channels << "!");
  }
  if(theAudioData.samples.size() == 0)
    return;
  else
    noWirelessSound.lastTimeOfIncomingSound = theFrameInfo.time;

  if(soundAmplitudes0.size() != sounds.size())
    soundAmplitudes0.resize(sounds.size());
  if(soundAmplitudes1.size() != sounds.size())
    soundAmplitudes1.resize(sounds.size());

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

  // Recognize the sound
  float  currentVolume = 0.0;
  if(inputChannel0.full() && numberOfNewAudioSamples >= SOUND_OVERLAP)
  {
    numberOfNewAudioSamples = 0;
    currentVolume  = computeCurrentVolume();
    checkForSounds(inputChannel0, soundAmplitudes0);
    checkForSounds(inputChannel1, soundAmplitudes1);
   // const bool vol = currentVolume > volumeThreshold;

    float bestAmplitude = -1;
    for(unsigned int i=0; i < sounds.size(); ++i)
    {
      const float thresh = sounds[i].threshold;
      if(soundAmplitudes0[i] > thresh || soundAmplitudes1[i] > thresh)
      {
        noWirelessSound.lastTimeSoundDetected = theFrameInfo.time;
        if(std::max(soundAmplitudes0[i], soundAmplitudes1[i]) > bestAmplitude)
        {
          bestAmplitude = std::max(soundAmplitudes0[i], soundAmplitudes1[i]);
          noWirelessSound.soundNumber = i;
        }
        if(outputFrequenciesWhenOverThresh)
        {
          OUTPUT_TEXT("vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv");
          OUTPUT_TEXT("Yay! I heard a sound with " << sounds[i].frequency << " Hz!  Amplitudes: " << soundAmplitudes0[i] << "    " << soundAmplitudes1[i]);
          int index = getSoundIndex(sounds[i].frequency);
          for(int i=index-2; i<=index+2; i++)
          {
            const double a = sqrt(sqr(fftDataIn[i][0]) + sqr(fftDataIn[i][1]));
            OUTPUT_TEXT("["<<i<<"] : "<<a);
          }
          OUTPUT_TEXT("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
        }
      }
    }
  }
  RECTANGLE("module:NoWirelessSoundRecognizer:frequencies", 0, 0, 4*1024, 100, 1, Drawings::solidPen, ColorRGBA::gray);
  COMPLEX_DRAWING("module:NoWirelessSoundRecognizer:frequencies")
  {
    for(unsigned int i = 0; i <= SOUND_BUFF_LEN; ++i)
    {
      double f = sqrt(sqr(fftDataIn[i][0]) + sqr(fftDataIn[i][1]));
      if(f > frequencyVisualizationCut)
      {
        RECTANGLE("module:NoWirelessSoundRecognizer:frequencies", i*4, 0, i*4+4, f, 1, Drawings::solidPen, ColorRGBA::black);
      }
    }
  }

  // Finally, plot the sound correlation of each channel:
  DECLARE_PLOT("module:NoWirelessSoundRecognizer:audioInput60HzChannel0");
  DECLARE_PLOT("module:NoWirelessSoundRecognizer:audioInput60HzChannel1");
  PLOT("module:NoWirelessSoundRecognizer:audioInput60HzChannel0", inputChannel0.back());
  PLOT("module:NoWirelessSoundRecognizer:audioInput60HzChannel1", inputChannel1.back());
  DECLARE_PLOT("module:NoWirelessSoundRecognizer:currentVolume");
  PLOT("module:NoWirelessSoundRecognizer:currentVolume", currentVolume);
}

void NoWirelessSoundRecognizer::checkForSounds(const RingBuffer<float, SOUND_BUFF_LEN>& inputChannel, std::vector<float>& amplitudes)
{
  const int REAL = 0;     // real value of complex
  const int IMAG = 1;     // imaginary value of complex

  for(int j = 0; j < SOUND_BUFF_LEN; j++)
  {
    soundInput8kHz[j] = inputChannel[j]; // write just the half of dataIn and the rest is zero padded
  }
  fftw_execute(fft2048); // compute the fft - repeat as needed
  
  for(unsigned int i=0; i<sounds.size(); i++)
  {
    const Sound& s = sounds[i];
    amplitudes[i] = -1.f;
    const int index = getSoundIndex(s.frequency);
    for(unsigned int j = index - s.tolerance; j <= index + static_cast<unsigned>(s.tolerance); ++j)
    {
      float a =static_cast<float>(sqrt(sqr(fftDataIn[j][REAL]) + sqr(fftDataIn[j][IMAG])));
      if(a > amplitudes[i])
        amplitudes[i] = a;
    }
  }
}

int NoWirelessSoundRecognizer::getSoundIndex(int frequency)
{
  if(frequency == 1000)
    return 256;
  else if(frequency == 2000)
    return 512;
  else if(frequency == 2500)
    return 640;
  else if(frequency == 3000)
    return 768;
  else if(frequency == 3500)
    return 896;
  else
    return static_cast<int>((frequency / 4000.f) * 1024.f);
}

float NoWirelessSoundRecognizer::computeCurrentVolume()
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

#endif
