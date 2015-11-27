/**
* @file Platform/Linux/Thread.h
*
* Declaration of a template class for threads and some
* other classes for synchronization. Based on code
* from B-Smart (real author unknown).
*
* @author <a href="mailto:Bernd.Gersdorf@dfki.de">Bernd Gersdorf</a>
*/

#pragma once

#include <pthread.h>
#ifdef OSX
#include <sched.h>
#endif
#include <unistd.h>
#include "Platform/BHAssert.h"
#include "Platform/Semaphore.h"

/**
* A class encapsulating a pthread
*/
template <class T> class Thread
{
private:
  Semaphore terminated; /**< Has the thread terminated? */
  pthread_t handle; /**< The pthread-handle */
  int priority; /**< The priority of the thread. */
  volatile bool running; /**< A flag which indicates the state of the thread */
  void (T::*function)(); /**< The address of the main function of the thread. */
  T* object; /**< A pointer to the object that is provided to the main function. */

  /**
  * The function is called when the thread is started.
  * It calls the main function of the thread as a member function of
  * an object.
  * @param p A pointer to the thread object.
  */
  static void* threadStart(Thread<T>* p)
  {
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, 0);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, 0);
    p->handle = pthread_self();
    ((p->object)->*(p->function))();
    p->running = false;
    p->terminated.post();
    return 0;
  }

public:
  /**
  * Default constructor.
  */
  Thread() : handle(0), running(false) {setPriority(0);}

  /**
  * Destructor.
  * Stops the thread, if it is still running.
  */
  virtual ~Thread() {stop();}

  /**
  * The function starts a member function as a new thread.
  * @param o The object the member function operates on.
  * @param f The member function.
  */
  void start(T* o, void (T::*f)())
  {
    if(running)
      stop();
    function = f;
    object = o;
    running = true;
    VERIFY(!pthread_create(&handle, 0, (void * (*)(void*)) &Thread<T>::threadStart, this));
    setPriority(priority);
  }

  /**
  * The function stops the thread.
  * It first signals its end by setting running to false. If the thread
  * does not terminate by itself, it will be killed after one second.
  */
  void stop()
  {
    if(handle)
    {
      running = false;
      terminated.wait(10000);
      pthread_cancel(handle);
      pthread_join(handle, 0);
      handle = 0;
    }
  }

  /**
  * The function announces that the thread shall terminate.
  * It will not try to kill the thread.
  */
  virtual void announceStop() {running = false;}

  /**
  * The function sets the priority of the thread.
  * @param prio Priority relative to "normal" priority.
  */
  void setPriority(int prio)
  {
    ASSERT(prio == 0 || (prio > 0 && prio <= sched_get_priority_max(SCHED_FIFO)));
    priority = prio;
    if(handle)
    {
      sched_param param;
      param.sched_priority = priority;
      VERIFY(!pthread_setschedparam(handle, priority == 0 ? SCHED_OTHER : SCHED_FIFO, &param));
    }
  }

  /**
  * The function determines whether the thread should still be running.
  * @return Should it continue?
  */
  bool isRunning() const {return running;}

  /**
  * The function returns the thread id.
  * @return The thread id. Only valid after the thread was started.
  */
  size_t getId() const {return (size_t) handle;}

  /**
  * The function returns the id of the calling thread.
  * @return The id of the calling thread.
  */
  static size_t getCurrentId() {return (size_t) pthread_self();}

  /**
    * Causes the calling thread to relinquish the CPU.
    */
  static void yield()
  {
#ifdef OSX
    sched_yield();
#else
    pthread_yield();
#endif
  }
};

/**
 * Names the current thread.
 * Not implemented on Linux (yet).
 * @param name The new name of the thread.
 */
#ifdef TARGET_ROBOT
#define NAME_THREAD(name) ((void) 0)
#elif defined(LINUX)
#define NAME_THREAD(name) VERIFY(!pthread_setname_np(pthread_self(), name + (strlen(name) > 15 ? strlen(name) - 15 : 0)))
#else
#define NAME_THREAD(name) VERIFY(!pthread_setname_np(name))
#endif

/**
* A class encapsulating a mutex lock.
*/
class SyncObject
{
private:
  pthread_mutex_t mutex; /**< The mutex. */

public:
  /**
  * Constructor.
  */
  SyncObject()
  {
    pthread_mutexattr_t attr;
    VERIFY(!pthread_mutexattr_init(&attr));
    VERIFY(!pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE));
    VERIFY(!pthread_mutex_init(&mutex, &attr));
    VERIFY(!pthread_mutexattr_destroy(&attr));
  }

  /**
  * Destructor
  */
  ~SyncObject()
  {
    VERIFY(!pthread_mutex_destroy(&mutex));
  }

  /**
  * The function enters the critical section.
  * It suspends the current thread, until the critical section
  * was left by all other threads.
  */
  void enter()
  {
    VERIFY(!pthread_mutex_lock(&mutex));
  }

  /**
  * The function leaves the critical section.
  */
  void leave()
  {
    VERIFY(!pthread_mutex_unlock(&mutex));
  }
};

/**
* The class provides a handy interface to using SyncObjects.
*/
class Sync
{
private:
  SyncObject& syncObject; /**< A reference to a sync object. */

public:
  /**
  * Constructor.
  * @param s A reference to a sync object representing a critical
  *          section. The section is entered.
  */
  Sync(SyncObject& s) : syncObject(s) {syncObject.enter();}

  /**
  * Destructor.
  * The critical section is left.
  */
  ~Sync() {syncObject.leave();}
};

/**
* The macro places a SyncObject as member variable into a class.
* This is the precondition for using the macro SYNC.
*/
#define DECLARE_SYNC mutable SyncObject _syncObject

/**
* The macro SYNC ensures that the access to member variables is synchronized.
* So only one thread can enter a SYNC block for this object at the same time.
* The SYNC is automatically released at the end of the current code block.
*/
#define SYNC Sync _sync(_syncObject)

/**
* The macro SYNC_WITH ensures that the access to the member variables of an
* object is synchronized. So only one thread can enter a SYNC block for the
* object at the same time. The SYNC is automatically released at the end of
* the current code block.
*/
#define SYNC_WITH(obj) Sync _sync((obj)._syncObject)
