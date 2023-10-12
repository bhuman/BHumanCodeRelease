/**
 * @file Platform/macOS/Semaphore.cpp
 * macOS implementation of class Semaphore for thread synchronization.
 * @author Thomas Röfer
 */

#include "Platform/BHAssert.h"
#include "Platform/Semaphore.h"
#include <dispatch/dispatch.h>
#include <pthread.h>

struct HandleQOSClass
{
  qos_class_t theClass;
  int priority;

  HandleQOSClass()
  {
    VERIFY(!pthread_get_qos_class_np(pthread_self(), &theClass, &priority));
    if(theClass != QOS_CLASS_UNSPECIFIED)
      VERIFY(!pthread_set_qos_class_self_np(QOS_CLASS_UTILITY, priority));
  }

  ~HandleQOSClass()
  {
    if(theClass != QOS_CLASS_UNSPECIFIED)
      VERIFY(!pthread_set_qos_class_self_np(theClass, priority));
  }
};

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
  HandleQOSClass handleQOSClass;
  return dispatch_semaphore_wait(static_cast<dispatch_semaphore_t>(handle), DISPATCH_TIME_FOREVER) == 0;
}

bool Semaphore::wait(unsigned timeout)
{
  HandleQOSClass handleQOSClass;
  return dispatch_semaphore_wait(static_cast<dispatch_semaphore_t>(handle), dispatch_time(DISPATCH_TIME_NOW, timeout * NSEC_PER_MSEC)) == 0;
}

bool Semaphore::tryWait()
{
  HandleQOSClass handleQOSClass;
  return dispatch_semaphore_wait(static_cast<dispatch_semaphore_t>(handle), DISPATCH_TIME_NOW) == 0;
}
