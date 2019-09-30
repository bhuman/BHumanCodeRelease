/**
 * @file Platform/Windows/SoundPlayer.h
 * The file declares a static class for playing sound files.
 * @author Colin Graf
 * @author Lukas Post
 */

#pragma once

#include "Platform/Semaphore.h"
#include "Platform/Thread.h"

#include <string>
#include <deque>

struct ISpVoice;

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
  ISpVoice* pVoice = nullptr;
  volatile bool closing;
  volatile bool playing;

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

  void playDirect(const std::string& name);

  void sayDirect(const std::string& text);
};
