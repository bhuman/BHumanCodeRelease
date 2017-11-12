/**
 * @file Platform/macOS/Semaphore.cpp
 * macOS implementation of class Semaphore for thread synchronization.
 * @author Thomas Röfer
 */

#include "Platform/BHAssert.h"
#include "Platform/Semaphore.h"
#define pi _pi
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wnullability-completeness-on-arrays"
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
