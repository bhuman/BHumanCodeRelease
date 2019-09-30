#include "Platform/Thread.h"

#include <Windows.h>

thread_local Thread* Thread::instance = nullptr;

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

void Thread::nameCurrentThread(const std::string& name)
{
  // Convert string to PCWSTR
  const int wchars_num = MultiByteToWideChar(CP_UTF8, 0, name.c_str(), -1, NULL, 0);
  wchar_t* pcwstr = new wchar_t[wchars_num];
  MultiByteToWideChar(CP_UTF8, 0, name.c_str(), -1, pcwstr, wchars_num);

  const HRESULT hr = SetThreadDescription(GetCurrentThread(), pcwstr);
  if(FAILED(hr))
  {
    // TODO Handle?
  }
  delete[] pcwstr;
}

const std::string Thread::getCurrentThreadName()
{
  std::string name;
  wchar_t* pcwstr;
  const HRESULT hr = GetThreadDescription(GetCurrentThread(), &pcwstr);
  if(SUCCEEDED(hr))
  {
    // Convert PCWSTR to string
    const int chars_num = WideCharToMultiByte(CP_UTF8, 0, pcwstr, -1, NULL, 0, NULL, NULL);
    char* charArray = new char[chars_num + 1];
    charArray[chars_num] = '\0';
    WideCharToMultiByte(CP_UTF8, 0, pcwstr, -1, charArray, chars_num, NULL, NULL);
    name = charArray;
    delete[] charArray;
  }
  else
  {
    // TODO Handle?
  }
  LocalFree(pcwstr);

  demangleThreadName(name);
  return name;
}
