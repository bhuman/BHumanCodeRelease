/**
 * @file Framework/ModuleContainer.h
 *
 * This file declares a thread that executes a container of modules.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Framework/Communication.h"
#include "Framework/Configuration.h"
#include "Framework/ModuleGraphRunner.h"
#include "Framework/ModulePacket.h"
#include "Framework/ThreadFrame.h"
#include <functional>
#include <list>
#include <string>

class Debug;
class FrameExecutionUnit;
struct Logger;
class LoggingController;
struct Settings;

/**
 * @class ModuleContainer
 *
 * A container that executes modules and shares representations with other containers.
 */
class ModuleContainer : public ThreadFrame
{
private:
  static thread_local std::list<std::function<bool(MessageQueue::Message message)>> messageHandlers; /**< A list of all MessageHandlers of this thread. */

  // Lists, since Sender.receiver would become invalid when resizing a vector.
  std::list<Receiver<ModulePacket>> receivers; /**< The list of all receivers of this thread. */
  std::list<Sender<ModulePacket>> senders; /**< The list of all senders of this thread. */

  const std::string name; /**< The name of this thread. */
  const int priority; /**< The priority of this thread. */

  FrameExecutionUnit* executionUnit = nullptr; /**< The thread specific code. */
  ModuleGraphRunner moduleGraphRunner; /**< The solution manager handles the execution of modules. */

  size_t originalSize = 0; /**< The size of the outgoing message queue at the begin of the frame. */
  size_t sizeAfterFrameBegin; /**< The size of the message queue after the first message was added. */
  Logger* logger; /**< Points to the only logger of this robot. */
  const LoggingController* loggingController = nullptr; /**< The control interface to the logger, owned by this module container if it controls the logger. */
  bool debugRequestWaiting = false; /**< Is a debug request waiting for this thread? */

public:
  /**
   * The constructor.
   * @param settings The settings of this module container.
   * @param robotName The name of the robot this module container belongs to.
   * @param config The initial configuration of all threads.
   * @param index The index of this thread in the config.
   */
  ModuleContainer(const Settings& settings, const std::string& robotName, const Configuration& config, const std::size_t index, Logger* logger);

  /** The destructor frees the execution unit. */
  ~ModuleContainer();

  /**
   * The function connects a sender and a receiver.
   *
   * @param sender The thread that should send messages to this thread.
   * @param config The configuration for the connections.
   */
  void connectWithSender(ModuleContainer* sender, const Configuration& config);

  /**
   * The function connects a thread with the debug thread.
   *
   * @param debug The debug thread.
   * @param config The configuration to set the size of the connections.
   */
  void connectWithDebug(Debug* debug, const Configuration::Thread& config);

  /**
   * The function adds a MessageQueue::Handler for this thread.
   *
   * @param messageHandler The message handler.
   */
  static void addMessageHandler(const std::function<bool(MessageQueue::Message message)>& messageHandler)
  {
    messageHandlers.emplace_back(messageHandler);
  }

protected:
  /**
   * The function returns the name of the thread.
   * @return The name of the thread.
   */
  const std::string getName() const override { return name; }

  /**
   * The function determines the priority of the thread.
   * @return The priority of the thread.
   */
  int getPriority() const override { return priority; }

  /**
   * The function is called once before the first frame. It should be used
   * for things that can't be done in the constructor.
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
  void terminate() override;

  /**
   * The function is called for every incoming debug message.
   *
   * @param message An interface to read the message from the queue.
   * @return Has the message been handled?
   */
  bool handleMessage(MessageQueue::Message message) override;
};
