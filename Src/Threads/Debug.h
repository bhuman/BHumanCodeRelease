/**
 * @file Threads/Debug.h
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
#include "Platform/DebugHandler.h"
#endif
#include "Tools/Framework/Configuration.h"
#include "Tools/Framework/ThreadFrame.h"
#include "Tools/Module/ModuleGraphCreator.h"

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

  std::string threadIdentifier; /**< The thread the messages from the GUI are meant to be sent to. */

  // Lists, since Sender.receiver would become invalid when resizing a vector.
  std::list<DebugReceiver<MessageQueue>> receivers; /**< The list of all receivers of this thread. */
  std::list<DebugSender<MessageQueue>> senders; /**< The list of all senders of this thread. */
  std::unordered_map<std::string, DebugSender<MessageQueue>*> senderMap;

  std::unique_ptr<ModuleGraphCreator> moduleGraphCreator; /**< Calculates the execution order of the modules of all threads and their data exchange. */
  Configuration config; /**< The inital configuration of all threads. */
  bool legacy = false; /**< Replaying a legacy log file? */

public:
  /**
   * The constructor.
   * @param config The inital configuration of all threads.
   */
  Debug(const Configuration& config);

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
  bool handleMessage(InMessage& message) override;

  friend class ModuleContainer; // To add receivers and senders
  friend class LocalRobot; // To add receiver and sender in simulation
};
