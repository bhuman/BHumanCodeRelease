/**
 * @file  Platform/Linux/SoundPlayer.h
 *
 * Declaration of class SoundPlayer.
 */

#pragma once

#include "Platform/Thread.h"
#include "Platform/Semaphore.h"

#include <alsa/asoundlib.h>

#include <deque>
#include <string>
#include <vector>

class SoundPlayer : public Thread
{
private:
  static SoundPlayer soundPlayer; /**< The only instance of this class. */
  DECLARE_SYNC;
  std::deque<std::string> queue;
  std::deque<std::vector<short>> sampleQueue;
  std::string filePrefix;
  bool started;
  Semaphore sem;
  volatile bool closing;

#ifdef TARGET_ROBOT
  snd_pcm_t* handle;

  unsigned retries = 10;      /**< Number of tries to open device. */
  unsigned retryDelay = 500;  /**< Delay before a retry to open device. */
  unsigned channels = 2;      /**< Number of channels to playback. */
  unsigned sampleRate = 8000; /**< Sample rate to playback. This variable will contain the framerate the driver finally selected. */
  snd_pcm_uframes_t periodSize = 512; //Frames per Period
#endif

public:
  /**
   * Put a filename into play sound queue.
   * If you want to play Config/Sounds/bla.wav use play("bla.wav");
   * @param name The filename of the sound file.
   * @return The amount of files in play sound queue.
   */
  static int play(const std::string& name);

  static int playSamples(std::vector<short>& samples);

private:
  SoundPlayer();

  ~SoundPlayer();

  /**
   * play all sounds in queue and block until finished.
   */
  void flush();

  /**
   * main function of this thread
   */
  void main();

  /**
   * start thread
   */
  void start();

  void playDirect(const std::string& basename);

  void playDirect(std::vector<short>& samples);
};
