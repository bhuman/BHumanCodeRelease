/**
 * @file BehaviorOption.h
 *
 * This file declares classes and macros to declare behavior options.
 * This is similar to declaring modules. However, behavior options
 * inherit from interfaces
 *
 * @author Arne Hasselbring (some parts borrowed from Src/Tools/Module.h)
 */

#pragma once

#include "Tools/Module/Module.h"

class BehaviorOptionInterface
{
public:
  /** Virtual destructor for polymorphism. */
  virtual ~BehaviorOptionInterface() = default;

  /** Executes the behavior. */
  virtual void execute() = 0;

protected:
  /** Modifies the parameters of the behavior. */
  virtual void modifyParameters() = 0;

  /** Is called every frame before any option is executed. */
  virtual void preProcess()
  {
  }

  /** Is called every frame after all options are executed. */
  virtual void postProcess()
  {
  }

  friend class BehaviorOptionRegistry;
};

class BehaviorOptionWithValue : public virtual BehaviorOptionInterface
{
public:
  /**
   * Returns how good it would be to execute this behavior now.
   * @return A value that indicates how good it would be to execute this behavior now.
   */
  virtual float value() const = 0;
};

class BehaviorOptionWithConditions : public virtual BehaviorOptionInterface
{
public:
  /**
   * Returns whether the behavior can be executed if it has not been executed in the previous frame.
   * @return Whether the behavior...
   */
  virtual bool preconditions() const = 0;

  /**
   * Returns whether the behavior can be executed if it has been executed in the previous frame.
   * @return Whether the behavior...
   */
  virtual bool invariants() const = 0;
};

class BehaviorOptionBase
{
private:
  static BehaviorOptionBase* first; /**< The head of the list of all behavior options. */
  BehaviorOptionBase* next; /**< The next entry in the list of all behavior options. */
  const char* name; /**< The name of the behavior option that can be created by this instance. */
  std::vector<const char*> (*getRequirements)(); /**< A function that returns the requirements of this behavior option. */

protected:
  /**
   * Abstract method to create an instance of a behavior option.
   * @return The address of the instance created.
   */
  virtual BehaviorOptionInterface* createNew() = 0;

public:
  /**
   * Constructor.
   * @param name The name of the behavior option that can be created by this instance.
   * @param getRequirements The function that returns the requirements.
   */
  BehaviorOptionBase(const char* name, std::vector<const char*> (*getRequirements)()) :
    next(first), name(name), getRequirements(getRequirements)
  {
    first = this;
  }
  /**
   * Adds the requirements of all behavior options to the info of a module
   * @param info The info to which the requirements are added.
   */
  static void addToModuleInfo(std::vector<ModuleBase::Info>& info);

  friend class BehaviorOptionRegistry;
};

template<typename O>
class BehaviorOption : public BehaviorOptionBase
{
private:
  /**
   * Creates a new instance of this option.
   * @return A pointer to the new instance.
   */
  BehaviorOptionInterface* createNew() override
  {
    return new O;
  }

public:
  /**
   * Constructor.
   * @param name The name of the behavior option.
   * @param getRequirements The function that returns the requirements.
   */
  BehaviorOption(const char* name, std::vector<const char*> (*getRequirements)()) :
    BehaviorOptionBase(name, getRequirements)
  {
  }
};

/**
 * Loads the parameters of a behavior option.
 * @param parameters The parameters.
 * @param behaviorOptionName The name of the behavior option (the same transformation as to module names is applied).
 * @param fileName A custom file name or nullptr.
 */
void loadBehaviorOptionParameters(Streamable& parameters, const char* behaviorOptionName, const char* fileName);

#define _BEHAVIOR_OPTION_PARAMETERS(x) _MODULE_JOIN(_BEHAVIOR_OPTION_PARAMETERS_, x)
#define _BEHAVIOR_OPTION_PARAMETERS_MODIFIES(type)
#define _BEHAVIOR_OPTION_PARAMETERS_EXECUTES(base, type)
#define _BEHAVIOR_OPTION_PARAMETERS_REQUIRES(type)
#ifdef _MSC_VER
#define _BEHAVIOR_OPTION_PARAMETERS__MODULE_DEFINES_PARAMETERS(header, ...) _MODULE_STREAMABLE(Params, Streamable, , header, __VA_ARGS__); using NoParameters = Params;
#define _BEHAVIOR_OPTION_PARAMETERS__MODULE_LOADS_PARAMETERS(header, ...) _MODULE_STREAMABLE(Params, Streamable, , header, __VA_ARGS__); using NoParameters = Params;
#else
#define _BEHAVIOR_OPTION_PARAMETERS__MODULE_DEFINES_PARAMETERS(header, ...) _STREAM_STREAMABLE(Params, Streamable, , header, __VA_ARGS__); using NoParameters = Params;
#define _BEHAVIOR_OPTION_PARAMETERS__MODULE_LOADS_PARAMETERS(header, ...) _STREAM_STREAMABLE(Params, Streamable, , header, __VA_ARGS__); using NoParameters = Params;
#endif

#define _BEHAVIOR_OPTION_LOAD(x) _MODULE_JOIN(_BEHAVIOR_OPTION_LOAD_, x)
#define _BEHAVIOR_OPTION_LOAD_MODIFIES(type)
#define _BEHAVIOR_OPTION_LOAD_EXECUTES(base, type)
#define _BEHAVIOR_OPTION_LOAD_REQUIRES(type)
#define _BEHAVIOR_OPTION_LOAD__MODULE_DEFINES_PARAMETERS(...)
#define _BEHAVIOR_OPTION_LOAD__MODULE_LOADS_PARAMETERS(...) loadBehaviorOptionParameters(*this, behaviorOptionName, fileName);

#define _BEHAVIOR_OPTION_DECLARE(x) _MODULE_JOIN(_BEHAVIOR_OPTION_DECLARE_, x)
#define _BEHAVIOR_OPTION_DECLARE_MODIFIES(type) public: type& the##type = BehaviorOptionRegistry::theInstance->the##type;
#define _BEHAVIOR_OPTION_DECLARE_EXECUTES(base, type) public: base& the##type = *BehaviorOptionRegistry::getOption<base>(#type);
#define _BEHAVIOR_OPTION_DECLARE_REQUIRES(type) public: const type& the##type = Blackboard::getInstance().alloc<type>(#type);
#define _BEHAVIOR_OPTION_DECLARE__MODULE_DEFINES_PARAMETERS(...)
#define _BEHAVIOR_OPTION_DECLARE__MODULE_LOADS_PARAMETERS(...)

#define _BEHAVIOR_OPTION_FREE(x) _MODULE_JOIN(_BEHAVIOR_OPTION_FREE_, x)
#define _BEHAVIOR_OPTION_FREE_MODIFIES(type)
#define _BEHAVIOR_OPTION_FREE_EXECUTES(base, type)
#define _BEHAVIOR_OPTION_FREE_REQUIRES(type) Blackboard::getInstance().free(#type);
#define _BEHAVIOR_OPTION_FREE__MODULE_DEFINES_PARAMETERS(...)
#define _BEHAVIOR_OPTION_FREE__MODULE_LOADS_PARAMETERS(...)

#define _BEHAVIOR_OPTION_REQUIREMENTS(x) _MODULE_JOIN(_BEHAVIOR_OPTION_REQUIREMENTS_, x)
#define _BEHAVIOR_OPTION_REQUIREMENTS_MODIFIES(type)
#define _BEHAVIOR_OPTION_REQUIREMENTS_EXECUTES(base, type)
#define _BEHAVIOR_OPTION_REQUIREMENTS_REQUIRES(type) _requirements.push_back(#type);
#define _BEHAVIOR_OPTION_REQUIREMENTS__MODULE_DEFINES_PARAMETERS(...)
#define _BEHAVIOR_OPTION_REQUIREMENTS__MODULE_LOADS_PARAMETERS(...)

#define _BEHAVIOR_OPTION_I(name, interface, n, ...) _BEHAVIOR_OPTION_II(name, interface, n, (_BEHAVIOR_OPTION_PARAMETERS, __VA_ARGS__), (_BEHAVIOR_OPTION_LOAD, __VA_ARGS__), (_BEHAVIOR_OPTION_DECLARE, __VA_ARGS__), (_BEHAVIOR_OPTION_FREE, __VA_ARGS__), (_BEHAVIOR_OPTION_REQUIREMENTS, __VA_ARGS__))

#define _BEHAVIOR_OPTION_II(theName, interface, n, params, load, declare, free, requirements) \
  namespace theName##BehaviorOption \
  { \
    _MODULE_ATTR_##n params \
    using Parameters = NoParameters; \
  } \
  class theName; \
  class theName##Base : public theName##BehaviorOption::Parameters, public interface \
  { \
  private: \
    using BaseType = theName##Base; \
    void modifyParameters() override \
    { \
      if(sizeof(NoParameters) < sizeof(theName##BehaviorOption::Parameters)) \
      { \
        Global::getDebugDataTable().updateObject("behavior:" #theName, *this, false); \
        DEBUG_RESPONSE_ONCE("debug data:behavior:" #theName) \
          OUTPUT(idDebugDataResponse, bin, "behavior:" #theName << TypeRegistry::demangle(typeid(theName##BehaviorOption::Parameters).name()) << *this); \
      } \
    } \
  public: \
    static std::vector<const char*> getRequirements() \
    { \
      std::vector<const char*> _requirements; \
      _MODULE_ATTR_##n requirements \
      return _requirements; \
    } \
  private: \
    _MODULE_ATTR_##n declare \
  public: \
    using Parameters = theName##BehaviorOption::Parameters; \
    theName##Base(const char* fileName = nullptr) \
    { \
      static const char* behaviorOptionName = #theName; \
      static_cast<void>(behaviorOptionName); \
      _MODULE_ATTR_##n load \
    } \
    ~theName##Base() \
    { \
      _MODULE_ATTR_##n free \
    } \
    theName##Base(const theName##Base&) = delete; \
    theName##Base& operator=(const theName##Base&) = delete; \
  };

/**
 * @brief Generates the behavior option's base class.
 * @param name The name of the behavior option.
 * @param interface The interface from which the option inherits.
 * @param header Normally only an opening brace.
 */
#define BEHAVIOR_OPTION(name, interface, header, ...) \
  _BEHAVIOR_OPTION_I(name, interface, _MODULE_TUPLE_SIZE(__VA_ARGS__), __VA_ARGS__)

/**
 * This macro creates a creator for the behavior option.
 * @param behaviorOption The name of the behavior option that can be created.
 */
#define MAKE_BEHAVIOR_OPTION(behaviorOption) \
  BehaviorOption<behaviorOption> the##behaviorOption##Option(#behaviorOption, behaviorOption##Base::getRequirements);

#include "BehaviorOptionRegistry.h"
