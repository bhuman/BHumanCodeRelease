/**
 * @file Platform/Semaphore.h
 * Declaration of class Semaphore for thread synchronization.
 * @author Colin Graf
 */

#pragma once

/**
 * Encapsulates an semaphore object.
 */
class Semaphore
{
private:
  void* handle; /**< The sem_t handle of the semaphore. */
#ifdef LINUX
  char handle2[sizeof(void*) * 4];
#endif

public:
  /**
   * Constructs an new semaphore object.
   * @param value The initial value for the semaphore.
   */
  Semaphore(unsigned value = 0);

  ~Semaphore();

  /**
   * Increments the semaphore counter.
   */
  void post();

  /**
   * Decrements the semaphore counter. This function returns immediatly if the
   * counter is greater than zero. Otherwise the wait call blocks until the semaphore
   * counter can be decremented.
   * @return Whether the decrementation was successful.
   */
  bool wait();

  /**
   * Decrements the semaphore counter. This function returns immediatly if the
   * counter is greater than zero. Otherwise the wait call blocks until the semaphore
   * counter can be decremented.
   * @param timeout A timeout for the blocking call. (in ms)
   * @return Whether the decrementation was successful.
   */
  bool wait(unsigned timeout);

  /**
   * Tries to decrement the semaphore counter. This function returns immediatly.
   * @return Whether the decrementation was successful.
   */
  bool tryWait();
};
