/**
 * @file Platform/Windows/Thread.h
 *
 * Declaration of template class Thread and classes for synchronization.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#define NOMINMAX
#include <windows.h>

/**
 * The class encapsulates a Windows thread.
 */
template<class T> class Thread
{
private:
  HANDLE handle; /**< The Windows handle of the thread. */
  DWORD id;
  int priority; /**< The priority of the thread. */
  volatile bool running; /**< A flag that states whether the thread is running. */
  void (T::*function)(); /**< The address of the main function of the thread. */
  T* object; /**< A pointer to the object that is provided to the main function. */

  /**
   * The function is called when the thread is started.
   * It calls the main function of the thread as a member function of
   * an object.
   * @param p A pointer to the thread object.
   */
  static unsigned long __stdcall threadStart(Thread<T>* p)
  {
    ((p->object)->*(p->function))();
    p->running = false;
    return 0;
  }

public:
  /**
   * Constructor.
   */
  Thread()
  {
    handle = 0;
    id = 0;
    running = false;
    setPriority(0);
  }

  /**
   * Destructor.
   * Stops the thread in case it is still running.
   */
  virtual ~Thread() {stop();}

  /**
   * The function starts a member function as a new thread.
   * @param o The object the member function operates on.
   * @param f The member function.
   */
  void start(T* o, void (T::*f)())
  {
    if(running) stop();
    function = f;
    object = o;
    running = true;
    handle = CreateThread(0, 0, (unsigned long(__stdcall*)(void*)) threadStart, this, 0, &id);
    SetThreadPriority(handle, priority);
  }

  /**
   * The function stops the thread.
   * It first signals its end by setting running to false. If the thread
   * does not terminate by itself, it will be killed after one second.
   */
  void stop()
  {
    running = false;
    if(handle && WaitForSingleObject(handle, 10000) == WAIT_TIMEOUT)
      TerminateThread(handle, 0);
  }

  /**
   * The function announces that the thread shall terminate.
   * It will not try to kill the thread.
   */
  virtual void announceStop() {running = false;}

  /**
   * The function suspends a thread.
   */
  void suspend() {SuspendThread(handle);}

  /**
   * The function resumes a suspended thread.
   */
  void resume() {ResumeThread(handle);}

  /**
   * The function sets the priority of the thread.
   * @param prio Priority relative to THREAD_PRIORITY_NORMAL.
   */
  void setPriority(int prio)
  {
    priority = prio + THREAD_PRIORITY_NORMAL;
    if(handle)
      SetThreadPriority(handle, priority);
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
  unsigned getId() const {return id;}

  /**
   * The function returns the id of the calling thread.
   * @return The id of the calling thread.
   */
  static unsigned getCurrentId() {return GetCurrentThreadId();}

  /**
   * Causes the calling thread to relinquish the CPU.
   */
  static void yield() {Sleep(0);}
};

/**
 * Names the current thread.
 * Not implemented on Windows (yet).
 * @param name The new name of the thread.
 */
#define NAME_THREAD(name) ((void) 0)

/**
 * The class encapsulates a critical section.
 */
class SyncObject
{
private:
  CRITICAL_SECTION section; /**< The Windows critical section. */

public:
  /**
   * Constructor.
   */
  SyncObject() {InitializeCriticalSection(&section);}

  /**
   * Destructor.
   */
  ~SyncObject() {DeleteCriticalSection(&section);}

  /**
   * The function enters the critical section.
   * It suspends the current thread, until the critical section
   * was left by all other threads.
   */
  void enter() {EnterCriticalSection(&section);}

  /**
   * The function leaves the critical section.
   */
  void leave() {LeaveCriticalSection(&section);}
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
#define DECLARE_SYNC SyncObject _syncObject

/**
 * The macro SYNC ensures that the access to member variables is synchronized.
 * So only one thread can enter a SYNC block for this object at the same time.
 * The SYNC is automatically released at the end of the current code block.
 * Never nest SYNC blocks, because this will result in a deadlock!
 */
#define SYNC Sync _sync(_syncObject)

/**
 * The macro SYNC_WITH ensures that the access to the member variables of an
 * object is synchronized. So only one thread can enter a SYNC block for the
 * object at the same time. The SYNC is automatically released at the end of
 * the current code block. Never nest SYNC blocks, because this will result
 * in a deadlock!
 */
#define SYNC_WITH(obj) Sync _sync((obj)._syncObject)
