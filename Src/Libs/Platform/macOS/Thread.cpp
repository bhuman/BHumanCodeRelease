#include "Platform/BHAssert.h"
#include "Platform/Thread.h"

#include <pthread.h>

static struct Helper
{
  int basePriority;

  Helper()
  {
    int policy;
    sched_param param;
    VERIFY(!pthread_getschedparam(pthread_self(), &policy, &param));
    basePriority = param.sched_priority;
  }
} helper;

thread_local Thread* Thread::instance = nullptr;

void Thread::stop(unsigned timeout)
{
  running = false;
  if(thread && thread->joinable())
  {
    if(!terminated.wait(timeout))
      pthread_cancel(thread->native_handle());
    thread->join();
    delete thread;
    thread = nullptr;
  }
}

void Thread::threadStart(const std::function<void()>& lambda)
{
  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, 0);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, 0);
  lambda();
  SYNC;
  running = false;
  terminated.post();
}

const std::string Thread::getCurrentThreadName()
{
  char cname[16];
  VERIFY(!pthread_getname_np(pthread_self(), cname, 16));
  std::string name(cname);
  demangleThreadName(name);
  return name;
}

void Thread::changePriority()
{
  SYNC;
  if(thread && running)
  {
    ASSERT((priority >= -2 && priority <= 0)
           || (priority > 0 && priority <= sched_get_priority_max(SCHED_FIFO)));
    sched_param param;
    param.sched_priority = priority + helper.basePriority;
    VERIFY(!pthread_setschedparam(thread->native_handle(), SCHED_FIFO, &param));
  }
}

void Thread::nameCurrentThread(const std::string& name)
{
  char cname[64] = "";
  name.copy(cname, sizeof(cname) - 1);
  cname[sizeof(cname) - 1] = '\0';
  VERIFY(!pthread_setname_np(cname));
}
