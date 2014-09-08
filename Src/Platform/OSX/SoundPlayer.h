/**
* @file  Platform/OSX/SoundPlayer.h
*
* Declaration of class SoundPlayer.
*/

#pragma once

#include <deque>
#include <string>
#include "Thread.h"
#include "Semaphore.h"

class SoundPlayer: Thread<SoundPlayer>
{
public:
  /**
   * Put a filename into play sound queue.
   * If you want to play Config/Sounds/bla.wav use play("bla.wav");
   * @param name The filename of the sound file.
   * @return The amound of files in play sound queue.
   */
  static int play(const std::string& name);

private:
  static SoundPlayer soundPlayer; /**< The only instance of this class. */

  /**
  * Destructor.
  */
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

  std::deque< std::string > queue;
  std::string filePrefix;
  DECLARE_SYNC;
  Semaphore sem;

  void playDirect(const std::string& basename);
};
