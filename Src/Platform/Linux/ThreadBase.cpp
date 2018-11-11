#include "Platform/Thread.h"
#include "Platform/BHAssert.h"

#include <pthread.h>

static DECLARE_SYNC;

void Thread::stop()
{
  running = false;
  if(thread && thread->joinable())
  {
    if(!terminated.wait(10000))
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

void Thread::changePriority()
{
  SYNC;
  if(thread && running)
  {
    ASSERT((priority >= -2 && priority <= 0)
           || (priority > 0 && priority <= sched_get_priority_max(SCHED_FIFO)));
    sched_param param;
    param.sched_priority = 0;
    switch(priority)
    {
      case -2:
#ifdef TARGET_ROBOT
        VERIFY(!pthread_setschedparam(thread->native_handle(), SCHED_IDLE, &param));
        break;
#endif
      case -1:
#ifdef TARGET_ROBOT
        VERIFY(!pthread_setschedparam(thread->native_handle(), SCHED_BATCH, &param));
        break;
#endif
      case 0:
        VERIFY(!pthread_setschedparam(thread->native_handle(), SCHED_OTHER, &param));
        break;
      default:
        param.sched_priority = priority;
        VERIFY(!pthread_setschedparam(thread->native_handle(), SCHED_FIFO, &param));
    }
  }
}
