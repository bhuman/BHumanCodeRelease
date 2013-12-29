/**
 * @file ModuleManager.h
 * Declaration of a class representing the module manager.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Module.h"
#include "Tools/Streams/AutoStreamable.h"
#include <map>
#include <vector>
#include <string>

class DefaultModule;

/**
 * @class ModuleManager
 * A class representing the module manager.
 */
class ModuleManager
{
private:
  /**
   * The class represents the current state of a module.
   */
  class ModuleState
  {
  public:
    ModuleBase* module; /**< A pointer to the module base that is able to create an instance of the module. */
    Blackboard* instance; /**< A pointer to the instance of the module if it was created. Otherwise the pointer is 0. */
    bool required; /**< A flag that is required when determining whether a module is currently required or not. */
    bool requiredBackup; /**< Temporary backup of "required" */

    /**
     * Constructor.
     * @param module A pointer to the module base that is able to create an instance of the module.
     */
    ModuleState(ModuleBase* module) : module(module), instance(0), required(false) {}

    /**
     * Comparison operator. Only uses the name for comparison.
     * @param name The module name this one is compared to.
     * @return Is this the name of the module?
     */
    bool operator==(const std::string& name) const {return module->name == name;}
  };

  /**
   * The class represents a provider of information, i.e. a method of a module that updates a representation.
   */
  class Provider
  {
  public:
    std::string representation; /**< The representation that will be provided. */
    ModuleState* moduleState; /**< The moduleState that will give access to the module that provides the information. */
    void (*update)(Blackboard&); /**< The update handler within the module. */
    void (*create)(); /**< The method to create a new instance of the representation. */
    void (*free)(); /**< The method to delete an instance of the representation. */

    /**
     * Constructor.
     * @param representation The name of the representation provided.
     * @param moduleState The moduleState that will give access to the module that provides the information.
     * @param update The update handler within the module.
     * @param create The create handler for the representation.
     * @param free The free handler for the representation.
     */
    Provider(const std::string& representation, ModuleState* moduleState,
             void (*update)(Blackboard&), void (*create)(), void (*free)())
    : representation(representation),
      moduleState(moduleState),
      update(update),
      create(create),
      free(free) {}

    /**
     * Comparison operator. Only uses the representation for comparison.
     * @param other The other provider this one is compared to.
     * @return Are the representation names the same?
     */
    bool operator==(const Provider& other) const {return representation == other.representation;}

    /**
     * Comparison operator. Only uses the representation for comparison.
     * @param representation The representation this one is compared to.
     * @return Does this provider provide the given representation?
     */
    bool operator==(const std::string& representation) const {return this->representation == representation;}
  };

  std::list<Provider> providers; /**< The list of providers that will be executed. */
  std::map<const char*, const char*> selected; /**< The providers selected. This is always the last configuration set. It may not work. */

  class Shared
  {
  public:
    std::string representation; /**< The name of the shared representation. */
    void (*create)(); /**< The method to create a new instance of the representation. */
    void (*free)(); /**< The method to delete an instance of the representation. */
    void (*out)(Out&); /**< The function that is able to write the representation to a stream. */
    void (*in)(In&); /**< The function that is able to read the representation from a stream. */

    /**
     * Constructor.
     * @param representation The name of the shared representation.
     */
    Shared(const std::string& representation) : representation(representation), create(0), free(0), out(0), in(0) {}

    /**
     * Comparison operator. Only uses the representation for comparison.
     * @param other The representaion name this one is compared to.
     * @return Are the representation names the same?
     */
    bool operator==(const std::string& other) const {return representation == other;}
  };

public:
  /**
   * The class is used for reading configurations.
   *
   * This class is public since it is used for the ModuleInfo also.
   */
  STREAMABLE(Configuration,
  {
  public:
    STREAMABLE(RepresentationProvider,
    {
    public:
      RepresentationProvider(const std::string& representation, const std::string& provider);

      /**
       * Comparison operator. Uses the representation name for comparison.
       * @param other The other RepresentationProvider this one is compared to.
       * @return Is this representations name lower than the other?
       */
      bool operator<(const RepresentationProvider& other) const
      {
        return (representation < other.representation);
      },

      (std::string) representation,
      (std::string) provider,
    }),

    (std::vector<RepresentationProvider>) representationProviders,
  });

private:
  std::list<ModuleState> modules; /**< The current state of all modules. */
  std::list<ModuleState> otherModules; /**< The modules in other processes. */
  std::list<Shared> shared; /**< The list of all shared representations. */
  unsigned timeStamp; /**< The timeStamp of the last module request. Communication is only possible if both sides use the same timestamp. */
  DefaultModule* defaultModule; /**< A module that can provide everything. */
  DefaultModule* otherDefaultModule; /**< The default module of other processes. */

  /**
   * Adds all representations that need to be shared between processes to the
   * attribute "shared".
   * @param config The module configuration that currently set up.
   * @return Everything ok? If not, the new configuration is invalid.
   */
  bool calcShared(const Configuration& config);

  /**
   * Adds all representations that need to be shared between processes to the
   * attribute "shared". This version of the is called for each configuration entry.
   * @param config The module configuration that currently set up.
   * @param representation The representation that should be provided.
   * @param module The module that should provide the representation.
   * @param modules All modules in the same process as "module".
   * @return Is the representation really provided by this module?
   */
  bool calcShared(const Configuration& config, const std::string& representation,
                  const ModuleState& module, const std::list<ModuleState>& modules);

  /**
   * The method brings the providers in the correct sequence.
   * @return Is the set of providers consistent?
   */
  bool sortProviders();

  /**
   * The method restores a previous module configuration.
   * It is called after it was determined that the new configuration is invalid.
   * @param providers The previous providers.
   * @param shared The previous list of shared represenations.
   */
  void rollBack(const std::list<Provider>& providers, const std::list<Shared>& shared);

public:
  /**
   * Constructor used when framework processes are mapped to threads.
   * In that case, each process sees all modules, because they are all declared in the
   * same address space. Therefore, this constructor filters them based on their categories.
   * @param categories The categories of modules executed by this process.
   * @param numOfCategories The number of categories.
   */
  ModuleManager(const char** categories, size_t numOfCategories);

  /**
   * Destructor.
   * Destructs all modules currently constructed.
   */
  ~ModuleManager();

  /**
   * The method loads the selection of solutions from a configuration file.
   */
  void load();

  /**
   * The method destroys all modules. It can be called to destroy the modules
   * before the constructor is called.
   */
  void destroy();

  /**
   * The method updates the list of the currently created modules.
   * @param stream The stream the new configuration is read from.
   * @param timeStamp The timeStamp of the last module request.
   */
  void update(In& stream, unsigned timeStamp = 0xffffffff);

  /**
   * The method executes all selected modules.
   */
  void execute();

  /**
   * The method reads a package from a stream.
   * @param stream A stream containing representations received from another process.
   */
  void readPackage(In& stream);

  /**
   * The method writes a package to a stream.
   * @param stream A stream that will be filled with representations that are sent
   *               to another process.
   */
  void writePackage(Out& stream) const;

  /**
   * Returns a vector containing the names of all provided representations.
   * The list is ordered by execution order.
   *
   * @note This call has O(N) complexity
   */
  std::vector<std::string> getCurrentRepresentatioNames() const;

  friend class DefaultModule; /**< Allowed to access local class ModuleState. */
};

/**
 * The class defines the default module that can provide everything.
 * The representations provided remain in their initial state.
 */
class DefaultModule : public ModuleBase
{
private:
  /**
   * The method normally creates an instance of the module.
   * However, there is no instance of the default module.
   * @return The address of the newly created instance.
   */
  Blackboard* createNew()
  {
    return 0;
  }

public:
  /**
   * Default constructor.
   */
  DefaultModule();

  /**
   * The method lets this module provide all the representations that any
   * of the other modules also provide.
   * @param modules The list of all module states.
   */
  void setRepresentations(const std::list<ModuleManager::ModuleState>& modules);
};
