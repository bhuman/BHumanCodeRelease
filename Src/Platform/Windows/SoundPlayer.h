/**
 * @file Platform/Windows/SoundPlayer.h
 * The file declares a static class for playing sound files.
 * @author Colin Graf
 */

#pragma once

#include <string>
#include <vector>

/**
 * @class SoundPlayer
 * The class provides a static function for playing sound files under Windows.
 */
class SoundPlayer
{
public:
  /**
   * Plays a sound file.
   * If you want to play Config/Sounds/bla.wav use "play("bla.wav");"
   * @param name The filename of the sound file.
   * @return The amount of files in play sound queue.
   */
  static int play(const std::string& name);

  /**
  * Stub for playing Sound Samples
  * @param name The samples.
  * @return The amount of files in play sound queue.
  */
  static int playSamples(std::vector<short>& samples);
};
