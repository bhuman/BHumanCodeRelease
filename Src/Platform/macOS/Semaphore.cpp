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
#include <limits>

Semaphore::Semaphore(unsigned value)
{
  VERIFY(MPCreateSemaphore(std::numeric_limits<long>::max(), value, reinterpret_cast<MPSemaphoreID*>(&handle)) == noErr);
}

Semaphore::~Semaphore()
{
  OSStatus status = MPDeleteSemaphore(static_cast<MPSemaphoreID>(handle));
  static_cast<void>(status);
  ASSERT(status == noErr || status == kMPTaskAbortedErr);
}

void Semaphore::post()
{
  MPSignalSemaphore(static_cast<MPSemaphoreID>(handle));
}

bool Semaphore::wait()
{
  return MPWaitOnSemaphore(static_cast<MPSemaphoreID>(handle), kDurationForever) == noErr;
}

bool Semaphore::wait(unsigned timeout)
{
  return MPWaitOnSemaphore(static_cast<MPSemaphoreID>(handle), kDurationMillisecond * timeout) == noErr;
}

bool Semaphore::tryWait()
{
  return MPWaitOnSemaphore(static_cast<MPSemaphoreID>(handle), kDurationImmediate) == noErr;
}
