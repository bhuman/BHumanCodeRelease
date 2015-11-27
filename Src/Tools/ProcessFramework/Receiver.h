/**
 * @file Tools/ProcessFramework/Receiver.h
 *
 * This file declares classes related to receivers.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Streams/InStreams.h"

class PlatformProcess;

/**
 * The class is the base class for receivers.
 * A receiver is an object that reads packages from another process.
 * The class manages a global list of all receivers in a process.
 */
class ReceiverList
{
private:
  ReceiverList* next = nullptr; /**< The successor of the current receiver. */
  std::string name;             /**< The name of a receiver without the module's name. */

protected:
  PlatformProcess* process;   /**< The process this receiver is associated with. */
  void* package[3];           /**< A triple buffer for received packages. */
  volatile int reading = 0;   /**< Index of package reserved for reading. */
  volatile int actual = 0;    /**< Index of package that is the most actual. */

  /**
   * The function checks whether a new package has arrived.
   */
  virtual void checkForPackage() = 0;

public:
  /**
   * The constructor.
   * @param process The process this receiver is associated with.
   * @param receiverName The connection name of the receiver without the process name.
   */
  ReceiverList(PlatformProcess* process, const std::string& receiverName);

  virtual ~ReceiverList();

  /**
   * Returns the begin of the list of all receivers.
   * @return The first receiver in the list, or 0 if the list ist empty.
   */
  ReceiverList*& getFirst();

  /**
   * Returns the next receiver in the list.
   * @return The next receiver in the list, or 0 if this receiver is the last one.
   */
  ReceiverList* getNext() const {return next;}

  /**
   * Returns the connection name of the receiver.
   * @return The connection name without the process name ("Receiver.type.O")
   */
  const std::string& getName() const {return name;}

  /**
   * The function checks for all receivers whether a new package has arrived.
   */
  void checkAllForPackages();

  /**
   * The function sets the package.
   * @param p The package.
   */
  void setPackage(void* p);

  /**
   * The function determines whether the receiver has a pending package.
   * @return Is there still an unprocessed package?
   */
  bool hasPendingPackage() const {return package[actual] != 0;}

  /**
   * The function searches for a receiver with the given name.
   * @param processName The name of the receiver's process.
   * @param receiverName The name of the requested receiver.
   * @return If the receiver is found, a pointer to it is returned.
   *         Otherwise, the funtion returns 0.
   */
  ReceiverList* lookup(const std::string& processName, const std::string& receiverName);
};

/**
 * The class implements a receiver.
 * A receiver is an object that reads packages from another process.
 */
template<class T> class Receiver : public ReceiverList, public T
{
private:
  /**
   * The function checks whether a new package has arrived.
   */
  virtual void checkForPackage()
  {
    reading = actual;
    if(package[reading])
    {
      T& data = *static_cast<T*>(this);
      InBinaryMemory memory(package[reading]);
      memory >> data;
      delete[] (char*)package[reading];
      package[reading] = 0;
    }
  }

public:
  /**
   * @param process The process this receiver is associated with.
   * @param receiverName The connection name of the receiver without the process name.
   */
  Receiver(PlatformProcess* process, const std::string& receiverName) :
    ReceiverList(process, receiverName)
  {}
};
