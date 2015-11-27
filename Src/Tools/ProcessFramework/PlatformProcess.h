/**
 * @file Tools/ProcessFramework/PlatformProcess.h
 *
 * This file declares the base class for processes.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Platform/Thread.h"
#include "Platform/SystemCall.h"
#include "Platform/Semaphore.h"

class SenderList;
class ReceiverList;
class ProcessBase;
template<class T> class ProcessFrame;

/**
 * This class is the base class for all processes.
 */
class PlatformProcess
{
private:
  DECLARE_SYNC;
  SenderList* firstSender = nullptr; /**< The begin of the list of all senders of this process. */
  ReceiverList* firstReceiver = nullptr; /**< The begin of the list of all receivers of this process. */
  int priority = 0; /**< The priority of the process. */
  Semaphore sem; /**< The semaphore is triggered whenever this process receives new data. */
  ProcessBase* processBase = nullptr;

protected:
  /**
   * The function will be call from the process framework in each frame.
   * It shall provide the main functionality of the process.
   * @return Should wait for external trigger?
   */
  virtual bool processMain() = 0;

public:
  virtual ~PlatformProcess() = default;

  /**
   * The function returns the begin of list of all senders.
   * Note that the function returns a reference that can be changed.
   * @return A reference to the address of the first element.
   */
  SenderList*& getFirstSender() {return firstSender;}

  /**
   * The function returns the begin of list of all receivers.
   * Note that the function returns a reference that can be changed.
   * @return A reference to the address of the first element.
   */
  ReceiverList*& getFirstReceiver() {return firstReceiver;}

  /**
   * The function sets the priority of the process.
   * @attention The priority can only be set in the constructor of the process.
   *            Any further changes will be ignored.
   * @param priority The new priority. Reasonable values are -2 .. 2 and 15.
   */
  void setPriority(int priority);

  /**
   * The function determines the priority of the process.
   * @return The priority of the process.
   */
  int getPriority() const {return priority;}

  /**
   * The method is called when the process is terminated.
   */
  virtual void terminate() {}

  /**
   * The method waits forever or until package was received.
   */
  void wait()
  {
    if(SystemCall::getMode() == SystemCall::physicalRobot)
      sem.wait(100);
    else 
      sem.wait();
  }

  /**
   * The method has to be called to announce the receiption of a package.
   */
  void trigger() {sem.post();}

  template<class T> friend class ProcessFrame; /**< For setting processBase. */
};
