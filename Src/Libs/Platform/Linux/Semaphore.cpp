/**
 * @file Platform/Linux/Semaphore.cpp
 * POSIX implementation of class Semaphore for thread synchronization.
 * @author Colin Graf
 */

#include "Platform/BHAssert.h"
#include "Platform/Semaphore.h"

#include <semaphore.h>
#include <cerrno>
#include <ctime>

Semaphore::Semaphore(unsigned int value)
{
  ASSERT(sizeof(handle2) >= sizeof(sem_t));
  handle = handle2;
  VERIFY(sem_init(static_cast<sem_t*>(handle), 0, value) != -1);
}

Semaphore::~Semaphore()
{
  VERIFY(sem_destroy(static_cast<sem_t*>(handle)) != -1);
}

void Semaphore::post()
{
  VERIFY(sem_post(static_cast<sem_t*>(handle)) != -1);
}

bool Semaphore::wait()
{
  if(sem_wait(static_cast<sem_t*>(handle)) == -1)
  {
    while(errno == 516 || errno == EINTR)
    {
      if(sem_wait(static_cast<sem_t*>(handle)) != -1)
        return true;
      //ASSERT(false);
    }
    return false;
  }
  return true;
}

bool Semaphore::wait(unsigned int timeout)
{
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_nsec += (timeout % 1000) * 1000000;
  ts.tv_sec += timeout / 1000 + ts.tv_nsec / 1000000000;
  ts.tv_nsec %= 1000000000;
  if(sem_timedwait(static_cast<sem_t*>(handle), &ts) == -1)
  {
    while(errno == 516 || errno == EINTR)
    {
      if(sem_timedwait(static_cast<sem_t*>(handle), &ts) != -1)
        return true;
      if(errno == ETIMEDOUT)
        break;
      //ASSERT(false);
    }
    ASSERT(errno == ETIMEDOUT);
    return false;
  }
  return true;
}

bool Semaphore::tryWait()
{
  return sem_trywait(static_cast<sem_t*>(handle)) != -1;
}
