/**
 * @file Platform/SystemCall.h
 */

#pragma once

/**
 * static class for system calls
 * @attention the implementation is system specific!
 */
class SystemCall
{
public:
  enum Mode
  {
    physicalRobot,
    remoteRobot,
    simulatedRobot,
    logFileReplay,
  };

  /** returns the name of the local machine*/
  static const char* getHostName();

  /** returns the first ip address of the local machine*/
  static const char* getHostAddr();

  /** returns the current execution mode */
  static Mode getMode();

  /** Returns the load and the physical memory usage in percent */
  static void getLoad(float& mem, float load[3]);

  /**
   * Returns the free disk space on a volume.
   * @param path A path to a directory or file on the volume.
   * @return The free disk space in bytes.
   */
  static unsigned long long getFreeDiskSpace(const char* path);

  /**
   * Disable/enable playback of future sounds.
   * @param isMuted Whether sounds are disabled.
   */
  static void mute(bool isMuted);

  /**
   * Put a filename into play sound queue.
   * If you want to play Config/Sounds/bla.wav use play("bla.wav");
   * @param name The filename of the sound file.
   * @return The amount of files in play sound queue.
   */
  static int playSound(const char* name);

  /**
   * Put a string to be synthesized to speech into play sound queue.
   * If you want the robot to say "Hello" use say("Hello").
   * @param text The string to be synthesized and played
   * @return The amount of elements in the play sound queue.
   */
  static int say(const char* text, float speed = 1.f);

  /**
   * Is the sound player currently playing a file?
   * @return Is currently a file replayed?
   */
  static bool soundIsPlaying();

  /**
   * Is a USB drive mounted?
   * @return Whether a USB drive is mounted.
   */
  static bool usbIsMounted();

private:
  static bool isMuted; /**< Whether sound playback requests will be ignored. */
};
