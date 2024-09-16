/**
 * @file Debug.h
 *
 * Declaration of class Debug.
 *
 * @author Martin Lötzsch
 * @author Jan Fiedler
 */

#pragma once

#define MAX_PACKAGE_SEND_SIZE 6000000
#define MAX_PACKAGE_RECEIVE_SIZE 3000000

#ifdef TARGET_ROBOT
#include "Framework/DebugHandler.h"
#endif
#include "Framework/Configuration.h"
#include "Framework/ModuleGraphCreator.h"
#include "Framework/ThreadFrame.h"

#include <unordered_map>

/**
 * @class Debug
 *
 * A thread for collection and distribution of debug messages.
 *
 * All messages from the threads to the PC are collected here and all messages from the
 * PC to the threads are distributed by the Debug thread.
 */
class Debug : public ThreadFrame
{
private:
#ifdef TARGET_ROBOT
  DebugHandler debugHandler;
#endif

  std::string threadName; /**< The thread the messages from the GUI are meant to be sent to. */
  std::string currentThreadName; /**< The thread the next message is sent to. */

  // Lists, since Sender.receiver would become invalid when resizing a vector.
  std::list<DebugReceiver<MessageQueue>> receivers; /**< The list of all receivers of this thread. */
  std::list<DebugSender<MessageQueue>> senders; /**< The list of all senders of this thread. */
  std::unordered_map<std::string, DebugSender<MessageQueue>*> senderMap;

  std::unique_ptr<ModuleGraphCreator> moduleGraphCreator; /**< Calculates the execution order of the modules of all threads and their data exchange. */
  Configuration config; /**< The initial configuration of all threads. */

  /**
   * Removes certain messages based on per-message-type criteria to reduce
   * the size of the queue. Some message types are kept, for some only the latest
   * messages per thread are kept, and for others only messages from the latest
   * frame are kept.
   */
  void removeRepetitions();

public:
  /**
   * The constructor.
   * @param settings The settings of this thread.
   * @param robotName The name of the robot this thread belongs to.
   * @param config The initial configuration of all threads.
   */
  Debug(const Settings& settings, const std::string& robotName, const Configuration& config);

protected:
  /**
   * The function determines the priority of the thread.
   * @return The priority of the thread.
   */
  int getPriority() const override { return 2; }

  /**
   * The function is called directly before the first call of main().
   */
  void init() override;

  /**
   * The main function of this thread, is called once in each frame.
   *
   * @return Should wait for external trigger?
   */
  bool main() override;

  /**
   * The function is called when the thread is terminated.
   */
  void terminate() override {}

  /**
   * Is called for every incoming debug message.
   * @param message the message to handle
   * @return if the message was handled
   */
  bool handleMessage(MessageQueue::Message message) override;

  friend class ModuleContainer; // To add receivers and senders
  friend class LocalConsole; // To add receiver and sender in simulation
};
