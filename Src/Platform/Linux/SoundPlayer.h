/**
 * @file  Platform/Linux/SoundPlayer.h
 *
 * Declaration of class SoundPlayer.
 */

#pragma once

#include "Platform/File.h"
#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Wave.h"

#include <alsa/asoundlib.h>
#include <deque>
#include <flite.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

struct SoundRequest {
  bool isTextToSpeech;
  float ttsDurationStretchFactor;
  std::string fileOrText;
  SoundRequest() = default;
  explicit SoundRequest(std::string fileName)
  : isTextToSpeech(false), ttsDurationStretchFactor(0.f), fileOrText(std::move(fileName))
  {};
  SoundRequest(std::string text, float stretchFactor)
  : isTextToSpeech(true), ttsDurationStretchFactor(stretchFactor), fileOrText(std::move(text))
  {};
  SoundRequest(const SoundRequest&) = default;
};

class SoundPlayer : public Thread
{
private:
  static SoundPlayer soundPlayer; /**< The only instance of this class. */
  DECLARE_SYNC;
  std::deque<SoundRequest> queue;
  std::string filePrefix;
  bool started;
  Semaphore sem;
  volatile bool closing;
  volatile bool playing;

  cst_voice *voice;
  std::unordered_map<std::string, Wave> synthesizedSounds;

  snd_pcm_t* handle;

  unsigned retries = 10;      /**< Number of tries to open device. */
  unsigned retryDelay = 500;  /**< Delay before a retry to open device. */
  unsigned sampleRate = 16000; /**< Sample rate to playback. This variable will contain the frame rate the driver finally selected. */
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
   * @param speed Use speed < 1 to talk slower and speed > 1 to talk faster.
   * @return The amount of elements in the play sound queue.
   */
  static int say(const std::string& text, float speed = 1.f);

  static bool isPlaying();

private:
  SoundPlayer();

  ~SoundPlayer();

  static int enqueue(const SoundRequest& soundRequest);

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
