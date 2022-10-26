/**
 * @file Platform/macOS/Semaphore.cpp
 * macOS implementation of class Semaphore for thread synchronization.
 * @author Thomas Röfer
 */

#include "Platform/BHAssert.h"
#include "Platform/Semaphore.h"
#include <dispatch/dispatch.h>

Semaphore::Semaphore(unsigned value)
{
  handle = dispatch_semaphore_create(value);
  ASSERT(handle);
}

Semaphore::~Semaphore()
{
  post();
  dispatch_release(static_cast<dispatch_semaphore_t>(handle));
}

void Semaphore::post()
{
  dispatch_semaphore_signal(static_cast<dispatch_semaphore_t>(handle));
}

bool Semaphore::wait()
{
  return dispatch_semaphore_wait(static_cast<dispatch_semaphore_t>(handle), DISPATCH_TIME_FOREVER) == 0;
}

bool Semaphore::wait(unsigned timeout)
{
  return dispatch_semaphore_wait(static_cast<dispatch_semaphore_t>(handle), dispatch_time(DISPATCH_TIME_NOW, timeout * NSEC_PER_MSEC)) == 0;
}

bool Semaphore::tryWait()
{
  return dispatch_semaphore_wait(static_cast<dispatch_semaphore_t>(handle), DISPATCH_TIME_NOW) == 0;
}
