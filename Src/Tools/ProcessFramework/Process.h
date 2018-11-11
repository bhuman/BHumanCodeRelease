/**
 * @file Process.h
 *
 * Contains the definition of class Process.
 */

#pragma once

#include "ProcessFramework.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/AnnotationManager.h"
#include "Tools/Debugging/DebugRequest.h"
#include "Tools/Debugging/DebugDataTable.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/TimingManager.h"
#ifdef TARGET_ROBOT
#include "Tools/AlignedMemory.h"
#endif

namespace asmjit
{
  class JitRuntime;
}

/**
 * @class Process
 *
 * System-independent base class for Processes.
 * The embedding into the system environment is done by system-dependent frameworks that use
 * derivates of the Process class.
 */
class Process : public PlatformProcess, public MessageHandler
#ifdef TARGET_ROBOT
  , public AlignedMemory
#endif
{
protected:
  AnnotationManager annotationManager; /**< keeps track of the annotations in this process */
  TimingManager timingManager; /**< keeps track of the module timing in this process */

private:
  MessageQueue& debugIn; /**< A queue for incoming debug messages. */
  MessageQueue& debugOut; /**< A queue for outgoing debug messages. */
  bool initialized; /**< A helper to determine whether the process is already initialized. */
  Blackboard blackboard; /**< The blackboard of this process. */
  Settings settings;
  DebugRequestTable debugRequestTable;
  DebugDataTable debugDataTable;
  DrawingManager drawingManager;
  DrawingManager3D drawingManager3D;
  asmjit::JitRuntime* asmjitRuntime;

public:
  /**
   * @param debugIn A reference to an incoming debug queue
   * @param debugOut A reference to an outgoing debug queue
   */
  Process(MessageQueue& debugIn, MessageQueue& debugOut);

  ~Process();

  /**
   * The main function is called from the process framework once in each frame.
   * It does the debug handling and calls the main function in the derivates.
   * @return Should wait for external trigger?
   */
  bool processMain() override;

  /**
   * The method initializes the pointers in class Global.
   */
  void setGlobals();

protected:
  /**
   * The main function is called once in each frame.
   * It must be implemented.
   * @return Should wait for external trigger?
   */
  virtual bool main() = 0;

  /**
   * That function is called once before the first main(). It can be used
   * for things that can't be done in the constructor.
   */
  virtual void init() {}

  /**
   * Is called for every incoming debug message.
   * @param message An interface to read the message from the queue
   * @return true if message was handled
   */
  bool handleMessage(InMessage& message) override;

  /**
   * Is called from within processMain() with the debugIn message queue.
   */
  virtual void handleAllMessages(MessageQueue& messageQueue);
};

/**
 * The base class for the debug package sender.
 * It only contains a flag that signals to all debug package senders that the current process
 * is terminating and they should abort blocking actions.
 */
class DebugSenderBase
{
protected:
  static bool terminating; /**< Is the current process terminating? */

  friend class RoboCupCtrl; /**< RoboCupCtrl will set this flag. */
};

/**
 * This template class implements a sender for debug packages.
 * It ensures that only a package is sent if it is not empty.
 */
template<typename T> class DebugSender : public Sender<T>, private DebugSenderBase
{
public:
  /**
   * The constructor.
   * @param process The process this sender is associated with.
   */
  DebugSender(PlatformProcess* process) : Sender<T>(process)
  {}

  /**
   * Marks the package for sending and transmits it to all receivers that already requested for it.
   * All other receiver may get it later if they request for it before the package is changed.
   * In function will only send a package if it is not empty.
   * @param block Whether to block when the packet cannot be send immediatly
   */
  void send(bool block = false)
  {
    if(!Sender<T>::isEmpty())
    {
      bool requestedNew = Sender<T>::requestedNew();
      if(block)
        while(!requestedNew && !terminating)
        {
          Thread::yield();
          requestedNew = Sender<T>::requestedNew();
        }
      if(requestedNew)
      {
        Sender<T>::send();
        Sender<T>::clear();
      }
    }
  }
};
