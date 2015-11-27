/**
* @file Platform/SimRobotQt/SystemCall.h
*
* Implementation of system calls and access to thread local storage.
* Only for use inside the simulator.
*/

#pragma once

#include <cstdlib>

/**
 * All process-local global variable declarations have to be preceeded
 * by this macro. Only variables of simple types can be defined, i.e.
 * no class instantiations are allowed.
 */
#ifdef WINDOWS
#define PROCESS_LOCAL __declspec(thread)
#else
#define PROCESS_LOCAL __thread
#endif

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
    teamRobot,
  };

  /** returns the current system time in milliseconds*/
  static unsigned getCurrentSystemTime();

  /** returns the real system time in milliseconds (never the simulated one)*/
  static unsigned getRealSystemTime();

  /**
  * The function returns the thread cpu time of the calling thread in microseconds.
  * return thread cpu time of the calling thread
  */
  static unsigned long long getCurrentThreadTime();

  /** returns the time since aTime*/
  static int getTimeSince(unsigned aTime)
  {
    return (int)(getCurrentSystemTime() - aTime);
  }

  /** returns the real time since aTime*/
  static int getRealTimeSince(unsigned aTime)
  {
    return (int)(getRealSystemTime() - aTime);
  }

  /** returns the name of the local machine*/
  static const char* getHostName();

  /** returns the first ip address of the local machine*/
  static const char* getHostAddr();

  /** Sleeps for some milliseconds.
  * \param ms The amout of milliseconds.
  */
  static void sleep(unsigned int ms);

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

  /** Allocate memory of given size with given alignment. */
  static void* alignedMalloc(size_t size, size_t alignment = 16);

  /** Free aligned memory.*/
  static void alignedFree(void* ptr);

#ifdef TARGET_SIM
  /**
   * Put a filename into play sound queue.
   * If you want to play Config/Sounds/bla.wav use play("bla.wav");
   * @param name The filename of the sound file.
   * @return The amound of files in play sound queue.
   */
  static int playSound(const char* name);
#endif
};
