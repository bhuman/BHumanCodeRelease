/**
 * @file  Platform/macOS/SoundPlayer.h
 *
 * Declaration of class SoundPlayer.
 */

#pragma once

#include <deque>
#include <string>
#include <vector>
#include "Platform/Semaphore.h"
#include "Platform/Thread.h"

class SoundPlayer : Thread
{
private:
  static SoundPlayer soundPlayer; /**< The only instance of this class. */
  DECLARE_SYNC;
  std::deque<std::string> queue;
  std::string filePrefix;
  Semaphore sem;
  bool playing = false;

public:
  /**
   * Put a filename into play sound queue.
   * If you want to play Config/Sounds/bla.wav use play("bla.wav");
   * @param name The filename of the sound file.
   * @return The number of elements in the play sound queue.
   */
  static int play(const std::string& name);

  /**
   * Put a string to be synthesized to speech into play sound queue.
   * If you want the robot to say "Hello" use say("Hello").
   * @param text The string to be synthesized and played
   * @return The amount of elements in the play sound queue.
   */
  static int say(const std::string& text);

  /**
   * Is the sound player currently playing a file?
   * @return Is currently a file replayed?
   */
  static bool isPlaying();

private:
  SoundPlayer();
  ~SoundPlayer();

  /**
   * The main function of this thread.
   */
  void main();

  /**
   * Starts the thread that plays the sounds.
   */
  void start();

  /**
   * Plays a single sound.
   * @param basename The filename of the sound file.
   */
  void playDirect(const std::string& basename);
};
