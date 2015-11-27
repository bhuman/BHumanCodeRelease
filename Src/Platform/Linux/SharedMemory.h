/**
 * @file Platform/linux/SharedMemory.h
 * A simplified interface to access shared memory for inter process communication.
 * @author <a href="afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <string>
#include <unistd.h>
#include <ctime>

/**
 * @class SharedMemory
 * A simplified interface to access shared memory for inter process communication.
 */
class SharedMemory
{
  /** The name of the shared memory. The semaphore's name is "/identifier". */
  std::string identifier;
  /** The size of the shared memory. */
  const size_t maxSize;

  /** File descriptor for the shared memory. */
  int fileDescriptor;
  /** Pointer to the shared memory block. */
  void* sharedMemory;
  /** Semaphore to protect shared memory from concurrent access. */
  sem_t* semaphore;

  /** Flag that indicates the successful initialization of the shared memory. */
  bool successful;
  /** Flag that indicates if the memory value is initialized. */
  bool initialized;

public:
  /**
   * Constructor.
   * @param identifier The name of the shared memory.
   * @param maxSize The size of the shared memory.
   */
  SharedMemory(const std::string& identifier, size_t maxSize);
  ~SharedMemory();

  /** Use another shared memory. */
  void reinitialize(const std::string& identifier);
  /** Closes the shared memory interface. */
  void closeAccess();
  /** @return Was the initialization successful? */
  bool success() { return successful; }
  /** @return Is the memory initialized? */
  bool memoryInitialized() { return initialized; }

  /** Initializes the shared memory. */
  template<class T>
  void initializeMemory(const T& initialValue)
  {
    T* pointer = (T*) sharedMemory;
    *pointer = initialValue;
    sem_post(semaphore);
    initialized = true;
  }

  /** Writes to the shared memory. */
  template<class T>
  SharedMemory& operator<<(const T& value)
  {
    sem_wait(semaphore);
    T* pointer = (T*) sharedMemory;
    *pointer = value;
    sem_post(semaphore);
    return *this;
  }

  /** Reads from the shared memory. */
  template<class T>
  SharedMemory& operator>>(T& value)
  {
    sem_wait(semaphore);
    value = *((T*) sharedMemory);
    sem_post(semaphore);
    return *this;
  }

  /**
   * Reads from the shared memory with a timed wait.
   * @return Success?
   */
  template<class T>
  bool nonBlockingRead(T& value, long nanoseconds)
  {
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts); // error ignored
    ts.tv_nsec += nanoseconds;
    bool success = sem_timedwait(semaphore, &ts) == 0;
    if(success)
    {
      value = *((T*) sharedMemory);
      sem_post(semaphore);
    }
    return success;
  }
};
