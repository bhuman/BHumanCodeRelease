/**
 * @file Tools/Framework/ModuleContainer.h
 *
 * This file declares a thread that executes a container of modules.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Framework/ThreadFrame.h"
#include "Framework/Configuration.h"
#include "Framework/ModuleGraphRunner.h"
#include "Framework/ModulePacket.h"
#include "Representations/Infrastructure/GameState.h"
#include <unordered_map>

class Debug;
class FrameExecutionUnit;
struct Logger;
struct Settings;

/**
 * @class ModuleContainer
 *
 * A container that executes modules and shares representations with other containers.
 */
class ModuleContainer : public ThreadFrame
{
private:
  static thread_local std::list<std::function<bool(InMessage& message)>> messageHandlers; /**< A list of all MessageHandlers of this thread. */

  // Lists, since Sender.receiver would become invalid when resizing a vector.
  std::list<Receiver<ModulePacket>> receivers; /**< The list of all receivers of this thread. */
  std::list<Sender<ModulePacket>> senders; /**< The list of all senders of this thread. */

  const std::string name; /**< The name of this thread. */
  const int priority; /**< The priority of this thread. */

  FrameExecutionUnit* executionUnit = nullptr; /**< The thread specific code. */
  ModuleGraphRunner moduleGraphRunner; /**< The solution manager handles the execution of modules. */

  int numberOfMessages = 0; /**< The number of debus messages at the beginning of a frame. */
  Logger* logger; /**< Points to the only logger of this robot. */

  GameState::State lastGameState = GameState::beforeHalf; /**< Game state in the last frame (for annotation). */

public:
  /**
   * The constructor.
   * @param settings The settings of this module container.
   * @param robotName The name of the robot this module container belongs to.
   * @param config The inital configuration of all threads.
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
   * The function adds a MessageHandler for this thread.
   *
   * @param messageHandler The message handler.
   */
  static void addMessageHandler(const std::function<bool(InMessage& message)>& messageHandler)
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
  bool handleMessage(InMessage& message) override;

private:
  /** A team number and the corresponding team name. */
  STREAMABLE(Team,
  {,
    (uint8_t)(0) number,
    (std::string) name,
  });

  /** The map from team numbers to team names. */
  STREAMABLE(TeamList,
  {,
    (std::vector<Team>) teams,
  });

  bool isLoggingMaster = false; /**< Whether this thread controls the logger. */
  std::unordered_map<uint8_t COMMA std::string> teams; /**< The map from team numbers to team names for naming the log file after the opponent. */

  /**
   * Builds a log file description matching the current game state.
   * @param gameState The current game state.
   * @return The description for the log file.
   */
  std::string getLogDescription(const GameState& gameState) const;
};
