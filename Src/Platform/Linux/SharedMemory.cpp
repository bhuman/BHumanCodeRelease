/**
 * @file Platform/linux/SharedMemory.cpp
 * A simplified interface to access shared memory for inter process communication.
 * @author <a href="afabisch@tzi.de">Alexander Fabisch</a>
 */
#include "SharedMemory.h"

SharedMemory::SharedMemory(const std::string& identifier, size_t maxSize) :
  identifier(identifier),
  maxSize(maxSize),
  fileDescriptor(-1),
  sharedMemory(MAP_FAILED),
  semaphore(SEM_FAILED),
  successful(true),
  initialized(false)
{
  reinitialize(identifier);
}

SharedMemory::~SharedMemory()
{
  closeAccess();
}

void SharedMemory::reinitialize(const std::string& identifier)
{
  closeAccess();

  this->identifier = identifier;
  successful = true;
  initialized = false;

  fileDescriptor = shm_open(identifier.c_str(), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
  if(fileDescriptor < 0)
  {
    successful = false;
    closeAccess();
  }

  if(ftruncate(fileDescriptor, maxSize) < 0)
  {
    successful = false;
    closeAccess();
  }

  sharedMemory = mmap(0, maxSize, PROT_WRITE, MAP_SHARED, fileDescriptor, 0);
  if(sharedMemory == MAP_FAILED)
  {
    successful = false;
    closeAccess();
  }

  semaphore = sem_open(("/" + identifier).c_str(), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR, 0);
  if(semaphore == SEM_FAILED)
  {
    successful = false;
    closeAccess();
  }
}

void SharedMemory::closeAccess()
{
  if(fileDescriptor != -1)
  {
    close(fileDescriptor);
    fileDescriptor = -1;
  }

  if(sharedMemory != MAP_FAILED)
  {
    munmap(sharedMemory, maxSize);
    sharedMemory = MAP_FAILED;
  }

  if(semaphore != SEM_FAILED)
  {
    sem_close(semaphore);
    semaphore = SEM_FAILED;
  }

  successful = false;
}
