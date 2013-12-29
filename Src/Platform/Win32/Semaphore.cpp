/**
* @file Platform/Win32/Semaphore.cpp
* Win32 implementation of class Semaphore for thread synchronization.
* @author Colin Graf
*/

#include "BHAssert.h"
#include "Semaphore.h"

#include <climits>
#include <windows.h>

Semaphore::Semaphore(unsigned int value)
{
  VERIFY(handle = CreateSemaphore(NULL, value, LONG_MAX, NULL));
}

Semaphore::~Semaphore()
{
  VERIFY(CloseHandle(handle));
}

void Semaphore::post()
{
  VERIFY(ReleaseSemaphore((HANDLE)handle, 1, 0));
}

bool Semaphore::wait()
{
  return WaitForSingleObject((HANDLE)handle, INFINITE) == WAIT_OBJECT_0;
}

bool Semaphore::wait(unsigned int timeout)
{
  return WaitForSingleObject((HANDLE)handle, timeout) == WAIT_OBJECT_0;
}

bool Semaphore::tryWait()
{
  return WaitForSingleObject((HANDLE)handle, 0) == WAIT_OBJECT_0;
}
