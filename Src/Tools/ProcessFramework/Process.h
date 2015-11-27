/**
 * @file Process.h
 *
 * Contains the definition of class Process and macros for receivers and senders.
 */

#pragma once

#include "ProcessFramework.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Settings.h"
#include "Tools/Streams/StreamHandler.h"
#include "Tools/Debugging/AnnotationManager.h"
#include "Tools/Debugging/DebugRequest.h"
#include "Tools/Debugging/DebugDataTable.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/TimingManager.h"
#if defined(TARGET_ROBOT)
#include "Tools/AlignedMemory.h"
#endif

/**
 * @class Process
 *
 * System-independent base class for Processes.
 * The embedding into the system environment is done by system-dependent frameworks that use
 * derivates of the Process class.
 */
class Process : public PlatformProcess, public MessageHandler
#if defined(TARGET_ROBOT)
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
  StreamHandler streamHandler;
  DrawingManager drawingManager;
  DrawingManager3D drawingManager3D;

public:
  /**
   * @param debugIn A reference to an incoming debug queue
   * @param debugOut A reference to an outgoing debug queue
   */
  Process(MessageQueue& debugIn, MessageQueue& debugOut);

  /**
   * The main function is called from the process framework once in each frame.
   * It does the debug handling and calls the main function in the derivates.
   * @return Should wait for external trigger?
   */
  bool processMain();

  /**
   * The method initializes the pointers in class Global.
   */
  void setGlobals();

protected:
  /**
   * The main funtion is called once in each frame.
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
  virtual bool handleMessage(InMessage& message);

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
class MultiDebugSenderBase
{
protected:
  static bool terminating; /**< Is the current process terminating? */

  friend class RoboCupCtrl; /**< RoboCupCtrl will set this flag. */
};

/**
 * This template class implements a sender for debug packages.
 * It ensures that only a package is sent if it is not empty.
 */
template<class T> class MultiDebugSender : public Sender<T>, private MultiDebugSenderBase
{
public:
  /**
   * The constructor.
   * @param process The process this sender is associated with.
   * @param name The connection name of the sender without the process name.
   */
  MultiDebugSender(PlatformProcess* process, const char* name) :
    Sender<T>(process, name)
  {}

  /**
   * Marks the package for sending and transmits it to all receivers that already requested for it.
   * All other receiver may get it later if they request for it before the package is changed.
   * In function will only send a package if it is not empty.
   * @param block Whether to block when the packet cannot be send immediatly
   */
  virtual void send(bool block = false)
  {
    if(!Sender<T>::isEmpty())
    {
      bool requestedNew = Sender<T>::requestedNew();
      if(block)
        while(!requestedNew && !terminating)
        {
          Thread<ProcessBase>::yield();
          requestedNew = Sender<T>::requestedNew();
        }
      if(requestedNew)
      {
        Sender<T>::send();
        Sender<T>::clear();
      }
    }
  }

  friend class RoboCupCtrl; // Access terminate
};

/**
 * This class implements a sender for MessageQueues.
 * It ensures that only a package is sent if it is not empty.
 */
class DebugSender : public MultiDebugSender<MessageQueue>
{
public:
  /**
   * @param process The process this sender is associated with.
   * @param name The connection name of the sender without the process name.
   */
  DebugSender(PlatformProcess* process, const char* name) :
    MultiDebugSender<MessageQueue>(process, name)
  {}
};

/**
 * The macro declares two debugging queues.
 * The macro shall be the first entry in the declaration of a process.
 */
#define DEBUGGING \
  Receiver<MessageQueue> theDebugReceiver; \
  DebugSender theDebugSender

/**
 * The macro initializes the two debugging queues and the base class.
 * The macro shall be the first entry after the colon in constructor
 * of the process.
 */
#define INIT_DEBUGGING \
  Process(theDebugReceiver,theDebugSender), \
  theDebugReceiver(this,"Receiver.MessageQueue.O"), \
  theDebugSender(this,"Sender.MessageQueue.S")

/**
 * The macro declares a receiver.
 * It must be used inside a declaration of a process, after the macro DEBUGGING.
 * @param type The type of the package. The variable actually declared has
 *             a type compatible to "type" and is called "thetypeReceiver".
 */
#define RECEIVER(type) \
  Receiver<type> the##type##Receiver

/**
 * The macro initializes a receiver for a certain type. It must be part of the
 * initializer list of the constructor of the process.
 * @param type The type of the package. The variable actually declared has
 *             a type compatible to "type" and is called "thetypeReceiver".
 */
#define INIT_RECEIVER(type) \
  the##type##Receiver(this,"Receiver." #type ".O")

/**
 * The macro declares a sender.
 * It must be used inside a declaration of a process, after the macro DEBUGGING.
 * @param type The type of the package. The variable actually declared has
 *             a type compatible to "type" and is called "thetypeSender".
 */
#define SENDER(type) \
  Sender<type> the##type##Sender

/**
 * The macro initializes a sender for a certain type. It must be part of the
 * initializer list of the constructor of the process.
 * @param type The type of the package. The variable actually declared has
 *             a type compatible to "type" and is called "thetypeSender".
 */
#define INIT_SENDER(type) \
  the##type##Sender(this,"Sender." #type ".S")

/**
 * The macro declares a receiver for a MessageQueue.
 * It must be used inside a declaration of a process, after the macro DEBUGGING.
 * It shall only be used in the Debug process.
 * @param source The source process of the package. The variable actually declared is
 *               of type MessageQueue and is called "thesourceReceiver".
 */
#define DEBUG_RECEIVER(source) \
  class Receive##source##MessageQueue : public MessageQueue {}; \
  Receiver<Receive##source##MessageQueue> the##source##Receiver

/**
 * The macro initializes a receiver for a MessageQueue. It must be part of the
 * initializer list of the constructor of the process.
 * @param source The source process of the package. The variable actually declared is
 *               of type MessageQueue and is called "thesourceReceiver".
 */
#define INIT_DEBUG_RECEIVER(source) \
  the##source##Receiver(this,#source "Receiver.MessageQueue.O") \

/**
 * The macro declares a sender for a MessageQueue.
 * It must be used inside a declaration of a process, after the macro DEBUGGING.
 * It shall only be used in the Debug process.
 * @param target The target process of the package. The variable actually declared is
 *               of type MessageQueue and is called "thetargetReceiver".
 */
#define DEBUG_SENDER(target) \
  class Send##target##MessageQueue : public MessageQueue {}; \
  MultiDebugSender<Send##target##MessageQueue> the##target##Sender

/**
 * The macro initializes a sender for a MessageQueue. It must be part of the
 * initializer list of the constructor of the process.
 * @param target The target process of the package. The variable actually declared is
 *               of type MessageQueue and is called "thetargetSender".
 */
#define INIT_DEBUG_SENDER(target) \
  the##target##Sender(this,#target "Sender.MessageQueue.S")
