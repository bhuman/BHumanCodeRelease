/**
 * @file Platform/Windows/Semaphore.cpp
 * Windows implementation of class Semaphore for thread synchronization.
 * @author Colin Graf
 */

#include "Platform/BHAssert.h"
#include "Platform/Semaphore.h"

#include <Windows.h>

#include <limits>

Semaphore::Semaphore(unsigned value)
{
  VERIFY(handle = CreateSemaphore(nullptr, value, std::numeric_limits<long>::max(), nullptr));
}

Semaphore::~Semaphore()
{
  VERIFY(CloseHandle(handle));
}

void Semaphore::post()
{
  VERIFY(ReleaseSemaphore(static_cast<HANDLE>(handle), 1, 0));
}

bool Semaphore::wait()
{
  return WaitForSingleObject(static_cast<HANDLE>(handle), INFINITE) == WAIT_OBJECT_0;
}

bool Semaphore::wait(unsigned timeout)
{
  return WaitForSingleObject(static_cast<HANDLE>(handle), timeout) == WAIT_OBJECT_0;
}

bool Semaphore::tryWait()
{
  return WaitForSingleObject(static_cast<HANDLE>(handle), 0) == WAIT_OBJECT_0;
}
