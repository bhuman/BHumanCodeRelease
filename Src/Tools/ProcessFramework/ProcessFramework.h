/**
 * @file Tools/ProcessFramework/ProcessFramework.h
 *
 * This file declares classes corresponding to the process framework.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include <list>
#include "Platform/Thread.h"
#include "PlatformProcess.h"
#include "Receiver.h"
#include "Sender.h"
#include "Tools/Streams/AutoStreamable.h"
#ifdef TARGET_SIM
#include "Controller/RoboCupCtrl.h"
#endif
#ifdef TARGET_ROBOT
#include "Tools/AlignedMemory.h"
#endif

/**
 * The class is a helper that allows to instantiate a class as an Windows process.
 * ProcessBase contains the parts that need not to be implemented as a template.
 * It will only be used by the macro MAKE_PROCESS and should never be used directly.
 */
class ProcessBase : public Thread
#ifdef TARGET_ROBOT
  , public AlignedMemory
#endif
{
protected:
  /**
   * The main function of this Windows thread.
   */
  virtual void main() = 0;

public:
  ~ProcessBase() = default;

  /**
   * The function starts the process by starting the Windows thread.
   */
  void start() {Thread::start(this, &ProcessBase::main);}

  /**
   * The functions searches for a sender with a given name.
   * @param name The name of the sender.
   * @return If the sender was found, a pointer to it is returned.
   *         Otherwise, the function returns 0.
   */
  virtual SenderList* lookupSender(const std::string& name) = 0;

  /**
   * The functions searches for a receiver with a given name.
   * @param name The name of the receiver.
   * @return If the receiver was found, a pointer to it is returned.
   *         Otherwise, the function returns 0.
   */
  virtual ReceiverList* lookupReceiver(const std::string& name) = 0;

  /**
   * The function returns the name of the process.
   * @return The name of the process that normally is its class name.
   */
  virtual const std::string& getName() const = 0;

  /**
   * The function returns a pointer to the process if it has the given name.
   * @param processName The name of the process that is searched for.
   * @return If the process has the required name, a pointer to it is
   *         returned. Otherwise, the function returns 0.
   */
  virtual PlatformProcess* getProcess(const std::string& processName) = 0;
};

/**
 * The class is a helper that allows to instantiate a class as an Windows process.
 * ProcessCreator contains the parts that need to be implemented as a template.
 * It will only be used by the macro MAKE_PROCESS and should never be used directly.
 */
template<typename T> class ProcessFrame : public ProcessBase
{
private:
  std::string name; /**< The name of the process. */
  T process; /**< The process. */

protected:
  /**
   * The main function of this Windows thread.
   */
  void main() override
  {
#ifdef TARGET_SIM
    Thread::nameThread(RoboCupCtrl::controller->getRobotName() + "." + name);
#else
    Thread::nameThread(name);
#endif

    // Call process.nextFrame if no blocking receivers are waiting
    setPriority(process.getPriority());
    process.processBase = this;
    Thread::yield(); // always leave processing time to other threads
    process.setGlobals();
    while(isRunning())
    {
      if(process.getFirstReceiver())
        process.getFirstReceiver()->checkAllForPackages();
      bool wait = process.processMain();
      if(process.getFirstSender())
        process.getFirstSender()->sendAllUnsentPackages();
      if(wait)
        process.wait();
    }
    process.terminate();
  }

public:

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wuninitialized"
#endif

  /**
   * Note that process.setGlobals() is called before process is constructed.
   * @param name The name of the process.
   */
  ProcessFrame(const std::string& name) : name((process.setGlobals(), name)) {}

#ifdef __clang__
#pragma clang diagnostic pop
#endif

  ~ProcessFrame()
  {
    process.setGlobals();
  }

  /**
   * The functions searches for a sender with a given name.
   * @param senderName The name of the sender.
   * @return If the sender was found, a pointer to it is returned.
   *         Otherwise, the function returns 0.
   */
  SenderList* lookupSender(const std::string& senderName) override
  {
    return process.getFirstSender() ? process.getFirstSender()->lookup(name, senderName) : nullptr;
  }

  /**
   * The functions searches for a receiver with a given name.
   * @param receiverName The name of the receiver.
   * @return If the receiver was found, a pointer to it is returned.
   *         Otherwise, the function returns 0.
   */
  ReceiverList* lookupReceiver(const std::string& receiverName) override
  {
    return process.getFirstReceiver() ? process.getFirstReceiver()->lookup(name, receiverName) : nullptr;
  }

  /**
   * The function returns the name of the process.
   * @return the name of the process.
   */
  const std::string& getName() const override {return name;}

  /**
   * The function returns a pointer to the process if it has the given name.
   * @param processName The name of the process that is searched for.
   * @return If the process has the required name, a pointer to it is
   *         returned. Otherwise, the function returns 0.
   */
  PlatformProcess* getProcess(const std::string& processName) override
  {
    if(name == processName)
      return &process;
    else
      return nullptr;
  }

  /**
   * The function announces that the thread shall terminate.
   * It will not try to kill the thread.
   */
  void announceStop() override
  {
    Thread::announceStop();
    process.trigger();
  }
};

class ProcessList;

/**
 * The class is a base class for process creators.
 */
class ProcessCreatorBase
{
private:
  static ProcessCreatorBase* first; /**< The head of the list of all process creators. */
  ProcessCreatorBase* next; /**< The next process creator in the list. */

protected:
  /**
   * The function creates a process.
   * @return A pointer to the new process.
   */
  virtual ProcessBase* create() const = 0;

public:
  ProcessCreatorBase() : next(first) {first = this;}
  virtual ~ProcessCreatorBase() = default;

  friend class ProcessList;
};

/**
 * The template class instantiates creators for processes of a certain type.
 */
template<typename T> class ProcessCreator : public ProcessCreatorBase
{
private:
  std::string name; /**< The name of the process that will be created. */

protected:
  /**
   * The function creates a process.
   * @return A pointer to the new process.
   */
  ProcessBase* create() const override {return new T(name);}

public:
  /**
   * @param name The name of the process that will be created.
   */
  ProcessCreator(const std::string& name) : name(name) {}
};

/**
 * The class implements a list of processes.
 */
class ProcessList : public std::list<ProcessBase*>
{
public:
  /**
   * Creates a process for each process constructor and inserts them
   * into the list.
   */
  ProcessList()
  {
    for(const ProcessCreatorBase* i = ProcessCreatorBase::first; i; i = i->next)
      push_back(i->create());
  }

  ~ProcessList()
  {
    for(iterator i = begin(); i != end(); ++i)
      delete *i;
  }

  /**
   * The function announces to all processes in the list that they should stop.
   */
  void announceStop()
  {
    for(iterator i = begin(); i != end(); ++i)
      (*i)->announceStop();
  }

  /**
   * The function waits for all processes in the list to stop.
   */
  void stop()
  {
    for(iterator i = begin(); i != end(); ++i)
      (*i)->stop();
  }

  /**
   * The function starts all processes in the list.
   */
  void start()
  {
    for(iterator i = begin(); i != end(); ++i)
      (*i)->start();
  }
};

STREAMABLE(ConnectionParameter,
{
  STREAMABLE(ProcessConnection,
  {,
    (std::string) sender,
    (std::string) receiver,
  }),

  (std::vector<ProcessConnection>) processConnections,
});

/**
 * The macro MAKE_PROCESS instantiates a process creator.
 * As a convention, it should be used in the last line of the
 * source file. For each process, MAKE_PROCESS must exactly be used
 * once.
 * @param className The type of the class that will later be instantiated
 *                 as a process.
 */
#define MAKE_PROCESS(className) \
  ProcessCreator<ProcessFrame<className>> _create##className(#className)
