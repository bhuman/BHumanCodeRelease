/**
 * @file ThreadFrame.h
 *
 * This file declares classes corresponding to the thread framework.
 *
 * @author Thomas Röfer
 * @author Jan Fiedler
 */

#pragma once

#include "Framework/Blackboard.h"
#include "Framework/Communication.h"
#include "Framework/Settings.h"
#include "Debugging/AnnotationManager.h"
#include "Debugging/Debugging.h"
#include "Debugging/DebugDataTable.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/DebugRequest.h"
#include "Debugging/TimingManager.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"

#include <list>

namespace asmjit
{
  inline namespace _abi_1_9
  {
    class JitRuntime;
  }
}

/**
 * @class ThreadFrame
 *
 * System-independent base class for threads.
 * The class is the toplevel processing unit for threads and implements base features such as the main routine.
 */
class ThreadFrame : public Thread
{
protected:
  DebugReceiver<MessageQueue>* debugReceiver = nullptr; /**< The MessageQueue for incoming debug messages. */
  DebugSender<MessageQueue>* debugSender = nullptr; /**< The MessageQueue for outgoing debug messages. */

private:
  Semaphore sem; /**< The semaphore is triggered whenever this thread receives new data. */

  AnnotationManager annotationManager; /**< Keeps track of the annotations in this thread. */
  Blackboard blackboard; /**< The blackboard of this thread. */
  Settings settings;
  DebugRequestTable debugRequestTable; /**< Currently active debug requests of this thread. */
  DebugDataTable debugDataTable; /**< The debug data table of this thread. */
  DrawingManager drawingManager;
  DrawingManager3D drawingManager3D;
  asmjit::JitRuntime* asmjitRuntime; /**< JIT and Remote Assembler for C++ in this thread. */
  TimingManager timingManager; /**< Keeps track of the module timing in this thread. */

protected:
  const std::string robotName; /**< The name of the robot this thread belongs to. */

public:
  /**
   * The constructor.
   * Used by the ModuleContainer and Debug in the simulation.
   * @param settings The settings of this thread.
   * @param robotName The name of the robot this thread belongs to.
   */
  ThreadFrame(const Settings& settings, const std::string& robotName);
  ~ThreadFrame();

  /**
   * The function starts the thread by starting the platform thread.
   */
  void start() { Thread::start(this, &ThreadFrame::threadMain); }

  /**
   * The function returns the name of the thread.
   * @return The name of the thread.
   */
  virtual const std::string getName() const { return TypeRegistry::demangle(typeid(*this).name()); }

  /**
   * The function initializes the pointers in class Global.
   */
  void setGlobals();

  /**
   * The function has to be called to announce the reception of a packet.
   */
  void trigger() { sem.post(); }

  /**
   * The function announces that the thread shall terminate.
   * It will not try to kill the thread.
   */
  void announceStop() override
  {
    Thread::announceStop();
    trigger();
  }

protected:
  /**
   * The constructor.
   * Used by the RobotConsole and by Debug on the robot.
   * Note: ThreadFrame cares about the destruction of the pointers.
   * Note: If no messaging is required pass <code>nullptr</code>.
   * @param settings The settings of this thread.
   * @param robotName The name of the robot this thread belongs to.
   * @param debugReceiver The receiver of this thread.
   * @param debugSender The sender of this thread.
   */
  ThreadFrame(const Settings& settings, const std::string& robotName, DebugReceiver<MessageQueue>* debugReceiver, DebugSender<MessageQueue>* debugSender);

  /**
   * The function determines the priority of the thread.
   *
   * @return The priority of the thread.
   */
  virtual int getPriority() const = 0;

  /**
   * The function is called once before the first frame. It should be used
   * for things that can't be done in the constructor.
   */
  virtual void init() = 0;

  /**
   * The main function of this thread, is called from the framework once in each frame.
   * It shall provide the main functionality of this thread.
   *
   * @return Should wait for external trigger?
   */
  virtual bool main() = 0;

  /**
   * The function is called when the thread is terminated.
   */
  virtual void terminate() = 0;

  /**
   * The function is called for every incoming debug message.
   *
   * @param message An interface to read the message from the queue.
   * @return Has the message been handled?
   */
  virtual bool handleMessage(MessageQueue::Message message);

  /**
   * Is called from within threadMain() with the debugReceiver message queue.
   */
  virtual void handleAllMessages(MessageQueue& messageQueue);

private:
  /**
   * The main function of this thread. Contains the main loop.
   * It does the debug handling and calls the main function in the derived class.
   */
  void threadMain();

  /**
   * The function waits forever or until packet was received.
   */
  void wait()
  {
    if(SystemCall::getMode() == SystemCall::physicalRobot)
      sem.wait(100);
    else
      sem.wait();
  }
};

/**
 * @class ThreadList
 *
 * The class implements a list of threads.
 */
class ThreadList : public std::list<ThreadFrame*>
{
public:
  ~ThreadList()
  {
    for(iterator i = begin(); i != end(); ++i)
      delete *i;
  }

  /**
   * The function announces to all threads in the list that they should stop.
   */
  void announceStop()
  {
    for(iterator i = begin(); i != end(); ++i)
      (*i)->announceStop();
  }

  /**
   * The function waits for all threads in the list to stop.
   */
  void stop()
  {
    for(iterator i = begin(); i != end(); ++i)
      (*i)->stop();
  }

  /**
   * The function starts all threads in the list.
   */
  void start()
  {
    for(iterator i = begin(); i != end(); ++i)
      (*i)->start();
  }

protected:
  /**
   * The function search for a thread with a given name.
   *
   * @param name The name of the thread.
   * @return The thread with the given name.
   */
  ThreadFrame* lookupThread(const std::string& name) const
  {
    for(const_iterator i = begin(); i != end(); ++i)
    {
      if((*i)->getName() == name)
        return *i;
    }
    OUTPUT_ERROR("ThreadFrame " << name << " not found");
    return nullptr;
  }
};
