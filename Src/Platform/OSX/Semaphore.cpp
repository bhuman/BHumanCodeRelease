/**
 * @file Platform/OSX/Semaphore.cpp
 * OS X implementation of class Semaphore for thread synchronization.
 * @author Thomas Röfer
 */

#include "BHAssert.h"
#include "Semaphore.h"
#define pi _pi
#include <CoreServices.h>
#undef pi

Semaphore::Semaphore(unsigned value)
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

bool Semaphore::wait(unsigned timeout)
{
  return MPWaitOnSemaphore((MPSemaphoreID) handle, kDurationMillisecond * timeout) == noErr;
}

bool Semaphore::tryWait()
{
  return MPWaitOnSemaphore((MPSemaphoreID) handle, kDurationImmediate) == noErr;
}
