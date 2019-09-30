/**
 * @file Tools/Module/ModuleGraphCreator.h
 *
 * Declaration of a class representing the module graph creator.
 *
 * @author Thomas Röfer
 * @author Jan Fiedler
 */

#pragma once

#include "Module.h"
#include "Tools/Framework/Configuration.h"
#include "Tools/Streams/TypeInfo.h"

#include <list>
#include <unordered_set>
#include <vector>

/**
 * @class ModuleGraphCreator
 *
 * A class that calculates the module order of all threads and their data exchange from a configuration.
 */
class ModuleGraphCreator
{
  friend class ModuleGraphCreatorTest; // Access for tests.
private:
  /**
   * The class represents a provider of information.
   */
  class Provider
  {
  public:
    const char* representation; /**< The representation that will be provided. */
    ModuleBase* moduleBase; /**< The module base that will give access to the module that provides the information. */

    /**
     * Constructor.
     * @param representation The name of the representation provided.
     * @param moduleBase The moduleBase that will give access to the module that provides the information.
     */
    Provider(const char* representation, ModuleBase* moduleBase) :
      representation(representation), moduleBase(moduleBase) {}
  };

  std::unordered_map<std::string, ModuleBase*> modules; /**< A map of all modules for quick access via name. */

  // The outer vector indicates the thread for which this is calculated.
  std::vector<std::vector<bool>> required; /**< Bitmap if a module is required. Uses modules index. Must not be changed after adding to the providers list. */
  std::vector<std::vector<std::vector<const char*>>> received; /**< The list of all names of representations received from other threads. */
  std::vector<std::vector<std::vector<const char*>>> sent; /**< The list of all names of representations sent to other threads */
  std::vector<std::list<Provider>> providers; /**< The list of providers of each thread that will be executed. */
  std::vector<std::string> representationsToReset; /**< The list of all representations that must be reset. */

public:
  Configuration config; /**< The last configuration set. It may not work. */
  TypeInfo typeInfo; /**< Information about all types. */

  /**
   * The constructor.
   * @param config The configuration of all threads.
   */
  ModuleGraphCreator(const Configuration& config);

  /**
   * The function calculates all new module configurations.
   * @param stream The stream the new configuration is read from.
   * @return Whether a valid module configuration is present.
   */
  bool update(In& stream);

private:
  /**
   * Find information about a representation provided or required by a certain module.
   * @param info The module info that is searched for the representation.
   * @param representation The name of the representation that is searched for.
   * @param required If false, it is only searched for representations that are provided by this module.
   *            Otherwise, it is only searched for presentations that are required.
   * @return Information found about the representation. End iterator if it was not found.
   */
  static std::vector<ModuleBase::Info>::const_iterator find(const std::vector<ModuleBase::Info>& info, const std::string& representation,
                                                            bool required = false);

  /**
   * Adds all representations that need to be shared between threads to the
   * attributes "sent" and "received".
   * @param config The module configuration that currently set up.
   * @return Everything ok? If not, the new configuration is invalid.
   */
  bool calcShared(const Configuration& config);

  /**
   * Adds all representations that need to be received by a thread to the
   * parameter "received". This version is called for each configuration entry.
   * @param config The module configuration that currently set up.
   * @param index The index of the calculated thread.
   * @param representation The representation that should be provided.
   * @param module The module that should provide the representation.
   * @param received The list of representations to update.
   * @return Is the representation really provided by this module?
   */
  bool calcShared(const Configuration& config, std::size_t index,
                  const std::string& representation, const ModuleBase* module,
                  std::vector<std::vector<const char*>>& received) const;

  /**
   * The function brings the providers in the correct sequence.
   * @param providedByDefault Representations and possible aliases provided by "default".
   * @param index The index of the thread to be calculated.
   * @return Is the set of providers consistent?
   */
  bool sortProviders(const std::vector<std::string>& providedByDefault, std::size_t index);

public: // Passing data to the public:
  /** Contains all the parameters a thread needs to run its modules. */
  STREAMABLE(ExecutionValues,
  {
    STREAMABLE(StringVector,
    {,
      (std::vector<std::string>) vector,
    });
    /** Represents a ModuleState. */
    STREAMABLE(ModuleRequired,
    {
      ModuleRequired() = default;
      ModuleRequired(const std::string& module, bool required) : module(module)COMMA required(required) {},

      (std::string) module,
      (bool)(false) required,
    });

    ExecutionValues() = default;
    ExecutionValues(std::vector<std::vector<const char*>>& received,  std::vector<std::vector<const char*>>& sent,
                    std::vector<std::string>& representationsToReset, std::vector<ModuleRequired>& modules,
                    std::vector<Configuration::RepresentationProvider>& providers),

    (std::vector<StringVector>) received, /**< Which data is received from which thread. */
    (std::vector<StringVector>) sent, /**< Which data is sent to which thread. */
    (std::vector<std::string>) representationsToReset, /**< All representations that must be reset. */
    (std::vector<ModuleRequired>) modules, /**< All available modules and whether they need to be executed. */
    (std::vector<Configuration::RepresentationProvider>) providers, /**< All active modules and the order in which they must be executed. */
  });

  /**
   * Returns the data that a thread needs to execute its modules.
   * @param index The index of the thread.
   * @return Data that a thread needs to execute its modules.
   */
  const ExecutionValues getExecutionValues(std::size_t index);
};
