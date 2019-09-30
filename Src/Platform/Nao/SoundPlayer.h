/**
 * @file  Platform/Linux/SoundPlayer.h
 *
 * Declaration of class SoundPlayer.
 */

#pragma once

#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Platform/File.h"

#include "flite.h"

#include <alsa/asoundlib.h>
#include <deque>
#include <string>
#include <vector>
#include <unordered_map>

class SoundPlayer : public Thread
{
private:
  static SoundPlayer soundPlayer; /**< The only instance of this class. */
  DECLARE_SYNC;
  std::deque<std::string> queue;
  std::string filePrefix;
  bool started;
  Semaphore sem;
  volatile bool closing;
  volatile bool playing;

  struct Wave
  {
    explicit Wave(File& file);
    explicit Wave(const cst_wave*);

    short channels;
    unsigned sampleRate;
    std::vector<short> data;
  };
  cst_voice *voice;
  std::unordered_map<std::string, Wave> synthesizedSounds;

  snd_pcm_t* handle;

  unsigned retries = 10;      /**< Number of tries to open device. */
  unsigned retryDelay = 500;  /**< Delay before a retry to open device. */
  unsigned sampleRate = 16000; /**< Sample rate to playback. This variable will contain the framerate the driver finally selected. */
  snd_pcm_uframes_t periodSize = 512; /**< Frames per period. */
  const float textToSpeechVolumeFactor = 1.4f; /** Increase text to speech volume by this factor (1.5 seems to be too high, results in cracking noise)  */

public:
  /**
   * Put a filename into play sound queue.
   * If you want to play Config/Sounds/bla.wav use play("bla.wav");
   * @param name The filename of the sound file.
   * @return The amound of elements in play sound queue.
   */
  static int play(const std::string& name);

  /**
   * Put a string to be synthesized to speech into play sound queue.
   * If you want the robot to say "Hello" use say("Hello").
   * @param text The string to be synthesized and played
   * @return The amount of elements in the play sound queue.
   */
  static int say(const std::string& text);

  static bool isPlaying();

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

  /**
   * Play back a wave object
   * @param wave Wave object to play back
   */
  void playWave(const Wave& wave);
};
