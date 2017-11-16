/**
 * @file Platform/Windows/SoundPlayer.h
 * The file declares a static class for playing sound files.
 * @author Colin Graf
 * @author Lukas Post
 */

#pragma once

#include "Platform/Thread.h"
#include "Platform/Semaphore.h"

#include <string>
#include <deque>

/**
 * @class SoundPlayer
 * The class provides a static function for playing sound files under Windows.
 */
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

public:
  /**
   * Plays a sound file.
   * If you want to play Config/Sounds/bla.wav use "play("bla.wav");"
   * @param name The filename of the sound file.
   * @return The amount of files in play sound queue.
   */
  static int play(const std::string& name);

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

  void playDirect(const std::string& name);
};
