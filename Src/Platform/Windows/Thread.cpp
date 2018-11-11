#include "Platform/Thread.h"

#include <Windows.h>

const DWORD MS_VC_EXCEPTION = 0x406D1388;

#pragma pack(push,8)
typedef struct tagTHREADNAME_INFO
{
  DWORD dwType; // Must be 0x1000.
  LPCSTR szName; // Pointer to name (in user addr space).
  DWORD dwThreadID; // Thread ID (-1=caller thread).
  DWORD dwFlags; // Reserved for future use, must be zero.
} THREADNAME_INFO;
#pragma pack(pop)

void Thread::stop()
{
  running = false;
  if(thread && thread->joinable())
  {
    if(!terminated.wait(10000))
      TerminateThread(thread->native_handle(), 0);
    thread->join();
    delete thread;
    thread = nullptr;
  }
}

void Thread::threadStart(const std::function<void()>& lambda)
{
  lambda();
  running = false;
  terminated.post();
}

void Thread::changePriority()
{
  if(thread && running)
    SetThreadPriority(thread->native_handle(), THREAD_PRIORITY_NORMAL + priority);
}

void Thread::nameThread(const std::string& name)
{
  THREADNAME_INFO info;
  info.dwType = 0x1000;
  info.szName = name.c_str();
  info.dwThreadID = GetCurrentThreadId();
  info.dwFlags = 0;

  __try
  {
    RaiseException(MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*)&info);
  }
  __except(EXCEPTION_EXECUTE_HANDLER) {}
}