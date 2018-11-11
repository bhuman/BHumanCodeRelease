#pragma once

#include "Semaphore.h"

#include <functional>
#include <mutex>
#include <string>
#include <thread>

class Thread
{
private:
  std::thread* thread = nullptr;
  std::thread::id id;
  bool running = false;
  int priority = 0;
  Semaphore terminated;

public:
  Thread() = default;
  Thread(int priority) : priority(priority) {}

  /**
   * Stops the thread in case it is still running.
   */
  virtual ~Thread() { stop(); }

  /**
   * The function starts a member function as a new thread.
   * @param o The object the member function operates on.
   * @param f The member function.
   */
  template<typename C>
  void start(C* o, void (C::*f)());

  /**
   * The function stops the thread.
   * It first signals its end by setting running to false. If the thread
   * does not terminate by itself, it will be killed after one second.
   */
  void stop();

  /**
   * The function announces that the thread shall terminate.
   * It will not try to kill the thread.
   */
  virtual void announceStop() { running = false; }

  /**
   * The function sets the priority of the thread.
   * @param prio The scheduling priority. Priorities > 0 use the real
   *             time scheduler, -2..0 uses the normal scheduler.
   */
  void setPriority(int prio) { priority = prio; changePriority(); }

  /**
   * The function determines whether the thread should still be running.
   * @return Should it continue?
   */
  bool isRunning() const { return running; }

  /**
   * The function returns the thread id.
   * @return The thread id. Only valid after the thread was started.
   */
  std::thread::id getId() const { return id; }

  /**
   * The function returns the id of the calling thread.
   * @return The id of the calling thread.
   */
  static std::thread::id getCurrentId() { return std::this_thread::get_id(); }

  /**
   * Causes the calling thread to relinquish the CPU.
   */
  static void yield() { std::this_thread::yield(); }

  static void sleep(unsigned ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

  static void nameThread(const std::string& name);

private:
  /**
   * The function is called when the thread is started.
   * It calls the main function of the thread as a member function of
   * an object.
   * @param p A pointer to the thread object.
   */
  void threadStart(const std::function<void()>& lambda);

  void changePriority();
};

template<typename C>
void Thread::start(C* o, void (C::*f)())
{
  if(thread)
    stop();

  running = true;
  thread = new std::thread(&Thread::threadStart, this, [o, f, this]() { id = getCurrentId(); (o->*f)(); });
  id = thread->get_id();
  changePriority();
}

/**
 * The macro places a std::recursive_mutex as member variable into a class.
 * This is the precondition for using the macro SYNC.
 */
#define DECLARE_SYNC std::recursive_mutex _mutex

/**
 * The macro SYNC ensures that the access to member variables is synchronized.
 * So only one thread can enter a SYNC block for this object at the same time.
 * The SYNC is automatically released at the end of the current code block.
 */
#define SYNC std::lock_guard<std::recursive_mutex> _lock(_mutex)

/**
 * The macro SYNC_WITH ensures that the access to the member variables of an
 * object is synchronized. So only one thread can enter a SYNC block for the
 * object at the same time. The SYNC is automatically released at the end of
 * the current code block.
 */
#define SYNC_WITH(obj) std::lock_guard<std::recursive_mutex> _lock((obj)._mutex)
