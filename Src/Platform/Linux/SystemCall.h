/**
* @file Platform/Linux/SystemCall.h
*
* Implementation of system calls and access to thread local storage.
* Only for use on Linux.
*
* @author <A href=mailto:brunn@sim.informatik.tu-darmstadt.de>Ronnie Brunn</A>
* @author <A href=mailto:risler@sim.informatik.tu-darmstadt.de>Max Risler</A>
*/

#pragma once

#include <cstdlib>

/**
* All process-local global variable declarations have to be preceeded
* by this macro. Only variables of simple types can be defined, i.e.
* no class instantiations are allowed.
*/
#define PROCESS_LOCAL __thread

/**
* static class for system calls
* @attention the implementation is system specific!
*/
class SystemCall
{
private:
  static unsigned base; /**< An offset used to convert Linux time to the time provided by this class. */

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

  /** returns an offset used to convert Linux time to the time provided by this class. */
  static unsigned getSystemTimeBase();

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

  /** returns the current execution mode */
  static Mode getMode();

  /** waits a certain number of ms. */
  static void sleep(unsigned ms);

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

  /**
   * Put a filename into play sound queue.
   * If you want to play Config/Sounds/bla.wav use play("bla.wav");
   * @param name The filename of the sound file.
   * @return The amound of files in play sound queue.
   */
  static int playSound(const char* name);
};
