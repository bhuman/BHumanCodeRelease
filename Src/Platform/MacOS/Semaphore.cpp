/**
* @file Platform/linux/Semaphore.cpp
* POSIX implementation of class Semaphore for thread synchronization.
* @author Colin Graf
*/

#include "BHAssert.h"
#include "Semaphore.h"

#define pi otherPi
#include "CoreServices.h"
#undef pi

Semaphore::Semaphore(unsigned int value)
{
  VERIFY(MPCreateSemaphore(1, value, (MPSemaphoreID*) &handle) == noErr);
}

Semaphore::~Semaphore()
{
  VERIFY(MPDeleteSemaphore((MPSemaphoreID) handle) == noErr);
}

void Semaphore::post()
{
  MPSignalSemaphore((MPSemaphoreID) handle);
}

bool Semaphore::wait()
{
  return MPWaitOnSemaphore((MPSemaphoreID) handle, kDurationForever) == noErr;
}

bool Semaphore::wait(unsigned int timeout)
{
  return MPWaitOnSemaphore((MPSemaphoreID) handle, kDurationMillisecond * timeout) == noErr;
}

bool Semaphore::tryWait()
{
  return MPWaitOnSemaphore((MPSemaphoreID) handle, kDurationImmediate) == noErr;
}
