/**
 * @file Platform/SystemCall.h
 */

#pragma once
#include <vector>
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
    logfileReplay,
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
   * Put a filename into play sound queue.
   * If you want to play Config/Sounds/bla.wav use play("bla.wav");
   * @param name The filename of the sound file.
   * @return The amount of files in play sound queue.
   */
  static int playSound(const char* name);

  /**
   * Is the sound player currently playing a file?
   * @return Is currently a file replayed?
   */
  static bool soundIsPlaying();
  
  static bool usbIsMounted();
};
