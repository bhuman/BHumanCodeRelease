/**
 * @file Tools/ProcessFramework/Sender.h
 *
 * This file declares classes related to senders.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Receiver.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/Streamable.h"

const int RECEIVERS_MAX = 20; /**< The maximum number of receivers connected to a single sender */

/**
 * The class is the base class for senders.
 * A sender is an object that sends packages to another process.
 * The class manages a global list of all senders in a process.
 */
class SenderList
{
private:
  SenderList* next = nullptr; /**< The successor of the current sender. */
  std::string name;           /**< The name of a sender without the module's name. */

protected:
  PlatformProcess* process;   /**< The process this sender is associated with. */

  /**
   * The function sends a package to all receivers that requested it.
   */
  virtual void sendPackage() = 0;

public:
  /**
   * @param process The process this sender is associated with.
   * @param senderName The connection name of the sender without the process name.
   */
  SenderList(PlatformProcess* process, const std::string& senderName);

  virtual ~SenderList() = default;

  /**
   * Returns the begin of the list of all senders.
   * @return The first sender in the list, or 0 if the list ist empty.
   */
  SenderList*& getFirst();

  /**
   * Returns the next sender in the list.
   * @return The next sender in the list, or 0 if this sender is the last one.
   */
  SenderList* getNext() const {return next;}

  /**
   * Returns the connection name of the sender.
   * @return The connection name without the process name ("Sender.type.O")
   */
  const std::string& getName() const {return name;}

  /**
   * The function must be called to finish the current frame.
   */
  void sendAllUnsentPackages();

  /**
   * The function adds a receiver to this sender.
   * @param r The receiver that is attached to this sender.
   */
  virtual void add(ReceiverList* r) = 0;

  /**
   * The function searches for a sender with the given name.
   * @param processName The name of the sender's process.
   * @param senderName The name of the requested sender.
   * @return If the sender is found, a pointer to it is returned.
   *         Otherwise, the function returns 0.
   */
  SenderList* lookup(const std::string& processName, const std::string& senderName);
};

/**
 * The class implements a sender.
 * A sender is an object that sends packages to an queue.
 */
template<typename T> class Sender : public SenderList, public T
{
protected:
  ReceiverList* receiver[RECEIVERS_MAX], /**< A list of all receivers. */
              * alreadyReceived[RECEIVERS_MAX]; /**< A list of all receivers that have already received the current package. */
  int numOfReceivers = 0, /**< The number of entries in the receiver list. */
      numOfAlreadyReceived = -1; /**< The number of entries in the already received list. */

  /**
   * The function adds a receiver to this sender.
   * @param r The receiver that is attached to this sender.
   */
  void add(ReceiverList* r) override {receiver[numOfReceivers++] = r;}

  /**
   * The function sends a package to all receivers that requested it.
   */
  void sendPackage() override
  {
    if(numOfAlreadyReceived != -1)
    {
      // send() has been called at least once
      int i;
      for(i = 0; i < numOfReceivers; ++i)
      {
        int j;
        for(j = 0; j < numOfAlreadyReceived; ++j)
          if(receiver[i] == alreadyReceived[j])
            break;
        if(j == numOfAlreadyReceived)
        {
          // receiver[i] has not received its requested package yet
          const T& data = *static_cast<const T*>(this);
          OutBinaryMemory stream(16384);
          stream << data;
          receiver[i]->setPackage(stream.obtainData());
          // note that receiver[i] has received the current package
          ASSERT(numOfAlreadyReceived < RECEIVERS_MAX);
          alreadyReceived[numOfAlreadyReceived++] = receiver[i];
        }
      }
    }
  }

public:
  /**
   * @param process The process this sender is associated with.
   */
  Sender(PlatformProcess* process) :
    SenderList(process, TypeRegistry::demangle(typeid(T).name()))
  {}

  /**
   * Returns whether a new package was requested from the sender.
   * This is always true if this is a blocking sender.
   * @return Has a new package been requested?
   */
  bool requestedNew() const
  {
    for(int i = 0; i < numOfReceivers; ++i)
      if(!receiver[i]->hasPendingPackage())
        return true;
    return false;
  }

  /**
   * Marks the package for sending and transmits it to all receivers that already requested for it.
   * All other receiver may get it later if they request for it before the package is changed.
   */
  void send()
  {
    numOfAlreadyReceived = 0;
    sendPackage();
  }
};
