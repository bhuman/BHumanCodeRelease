/**
 * @file Tools/Module/ModuleGraphRunner.h
 *
 * Declaration of a class representing the module graph runner.
 *
 * @author Thomas Röfer
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/Framework/Configuration.h"
#include "Tools/Module/ModuleGraphCreator.h"

#include <vector>

class In;
class Out;

/**
 * @class ModuleGraphRunner
 *
 * A class that executes the modules of a thread.
 */
class ModuleGraphRunner
{
private:
  /**
   * The class represents the current state of a module.
   */
  class ModuleState
  {
  public:
    ModuleBase* module; /**< A pointer to the module base that is able to create an instance of the module. */
    Streamable* instance = nullptr; /**< A pointer to the instance of the module if it was created. Otherwise the pointer is 0. */
    bool required = false; /**< A flag that is required when determining whether a module is currently required or not. */

    /**
     * Constructor.
     * @param module A pointer to the module base that is able to create an instance of the module.
     */
    ModuleState(ModuleBase* module) : module(module) {}
  };

  /**
   * The class represents a provider of information, i.e. a method of a module that updates a representation.
   */
  class Provider
  {
  public:
    const char* representation; /**< The representation that will be provided. */
    ModuleState* moduleState; /**< The moduleState that will give access to the module that provides the information. */
    void (*update)(Streamable&); /**< The update handler within the module. */

    /**
     * Constructor.
     * @param representation The name of the representation provided.
     * @param moduleState The moduleState that will give access to the module that provides the information.
     * @param update The update handler within the module.
     */
    Provider(const char* representation, ModuleState* moduleState, void (*update)(Streamable&)) :
      representation(representation), moduleState(moduleState), update(update)
    {}
  };

  std::unordered_map<std::string, ModuleBase*> allModules; /**< A map of all modules for quick access via name. */
  bool validConfiguration = false;

  std::unordered_map<std::string, ModuleState> modules; /**< The current state of all available modules. Must not be changed after adding to the providers list. */
  std::vector<ModuleGraphCreator::ExecutionValues::StringVector> received; /**< The list of all names of representations received from other threads. */
  std::vector<ModuleGraphCreator::ExecutionValues::StringVector> sent; /**< The list of all names of representations sent to other threads */

  std::list<Provider> providers; /**< The list of providers that will be executed. */
  std::vector<std::vector<Streamable*>> toReceive; /**< The list of all representations received from other threads. */
  std::vector<std::vector<Streamable*>> toSend; /**< The list of all representations sent to other threads. */

  unsigned timestamp = 0; /**< The timestamp of the last module request. Communication is only possible if both sides use the same timestamp. */
  unsigned nextTimestamp = 0; /**< The next timestamp used to verify communication. */

public:
  /**
   * The constructor.
   * @param numberOfThreads The number of threads.
   */
  ModuleGraphRunner(size_t numberOfThreads) : toReceive(numberOfThreads), toSend(numberOfThreads)
  {
    for(ModuleBase* i = ModuleBase::first; i; i = i->next)
      allModules.emplace(i->name, i);
  }

  /**
   * Destructor.
   * Destructs all modules currently constructed.
   */
  ~ModuleGraphRunner() { destroy(); };

  /**
   * Returns whether a valid module configuration is present.
   * @return Whether a valid module configuration is present.
   */
  bool isValid() const { return validConfiguration; }

  /**
   * Returns whether the module configuration has changed, but the
   * current modules do not reflect that change yet.
   * @return Whether the module configuration has changed.
   */
  bool hasChanged() const { return !timestamp; }

  /**
   * The function destroys all modules. It can be called to destroy the modules
   * before the destructor is called.
   */
  void destroy();

  /**
   * The function updates to the new configuration.
   * @param stream The stream the new configuration is read from.
   */
  void update(In& stream);

  /**
   * The function executes all selected modules.
   */
  void execute();

  /**
   * The function reads a packet from a stream.
   * @param stream A stream containing representations received from another thread.
   * @param index The index of the thread this packet is from.
   */
  void readPacket(In& stream, const std::size_t index);

  /**
   * The function writes a packet to a stream.
   * @param stream A stream that will be filled with representations that are sent
   *               to another thread.
   * @param index The index of the thread this packet is for.
   */
  void writePacket(Out& stream, const std::size_t index) const;

  /**
   * The function checks whether no data would be received in a packet from a
   * certain thread.
   * @param index The index of the thread the packet would be from.
   */
  bool receiverEmpty(const std::size_t index) const
  {
    return toReceive[index].empty();
  }

  /**
   * The function checks whether no data would be sent in a packet to a
   * certain thread.
   * @param index The index of the thread the packet would be for.
   */
  bool senderEmpty(const std::size_t index) const
  {
    return toSend[index].empty();
  }
};
