/**
 * @file Cabsl.h
 *
 * The file defines a set of macros that define the C-based agent
 * behavior specification language (CABSL). Semantically, it follows
 * the ideas of XABSL.
 *
 * Grammar:
 *
 *     <cabsl>       = { <option> }
 *
 *     <option>      =  option '(' [ '(' <C-ident> ')' ] <C-ident>
 *                      [ ',' args '(' <decls> ')' ]
 *                      [ ',' ( defs | load ) '(' <decls> ')' ]
 *                      [ ',' vars '(' <decls> ')' ]
 *                      ')' ( <body> | ';' )
 *
 *     <body>        = '{'
 *                     [ <C-statements> ]
 *                     [ common_transition <transition> ]
 *                     { <other-state> } initial_state <state> { <other-state> }
 *                     '}'
 *
 *     <decls>       = <decl> { ',' <decl> }
 *
 *     <decl>        = '(' <type> ')' [ '(' <C-expr> ')' ] ' ' <C-ident>
 *
 *     <other-state> = ( state | target_state | aborted_state ) <state>
 *
 *     <state>       = '(' C-ident ')'
 *                     '{'
 *                     [ transition <transition> ]
 *                     [ action <action> ]
 *                     '}'
 *
 *     <transition>  = '{' <C-ifelse> '}'
 *
 *     <action>      = '{' <C-statements> '}'
 *
 * `<C-ident>` is a normal C++ identifier. `<C-expr>` is a normal C++
 * expression that can be used as default value for an argument.
 * `<C-ifelse>` is a decision tree. It should contain `goto` statements
 * (names of states are labels).  Conditions can use the pre-defined
 * symbols `state_time`, `option_time`, `action_done`, and
 * `action_aborted`. Within a state, the action `<C-statements>` can contain
 * calls to other (sub)options. `action_done` determines whether the last
 * sub-option called reached a target state in the previous execution cycle.
 * `action_aborted` does the same for an aborted state. At the beginning of an
 * option, it is possible to add arbitrary C++ code, which can contain
 * definitions that are shared by all states, e.g. lambda functions that
 * contain calculations used by more than one state. It is not allowed to call
 * other options outside of `action` blocks.
 *
 * Options can declare three kinds of additional parameters:
 *
 * - Arguments (`args`) that can be passed to the option.
 * - Definitions (`defs`, `load`) define constant parameters of the option, i.e.
 *   usually values the implementation depends upon. If `load` is used
 *   instead of `defs`, their values are loaded from a configuration file with
 *   the name `option<option name>.cfg`.
 * - Variables (`vars`) define option-local variables. They keep their
 *   values from each call of the option to the next call. However, they
 *   are (re-)initialized with their default values if the option was not
 *   executed in the previous cycle.
 *
 * Options can either be defined inline or just be declared. In case of the
 * latter, there also must be an implementation in a separate file that
 * includes the file containing the declaration. In that file, the name of
 * the option must be preceded by the name of its class in parentheses.
 *
 * If an option is defined inline, its head can contain arguments,
 * definitions, and variables. If it is split into a declaration and an
 * actual implementation, the declaration can only contain the arguments
 * (with optional default values) while the implementation must repeat the
 * arguments (without default values) and it can contain definitions and
 * variables.
 *
 * If options with arguments are called, a single struct is passed
 * containing all arguments. This means that the list of actual arguments
 * has to be enclosed in braces. This allows to specify their names in the
 * call, e.g. `go_to_point({.x = 10, .y = 20})`. Arguments not specified
 * are set to their default values, either the ones that were given in the
 * argument list of the option or the default value for the datatype.
 *
 * The command `select_option` allows to execute one option from a list of
 * options. It is tried to execute each option in the list in the sequence they
 * are given. If an option determines that it cannot currently be executed,
 * it stays in its `initial_state`. Otherwise, it is run normally. `select_option`
 * stops after the first option that was actually executed. Note that an
 * option that stays in its `initial_state` when it was called by `select_option`
 * is considered as not having been executed at all if it has no action block
 * for that state. If it has, the block is still executed, but neither the
 * `option_time` nor the `state_time` are increased.
 *
 * If Microsoft Visual Studio is used and options are included from separate
 * files, the following preprocessor code might be added before including
 * this file. `Class` has to be replaced by the template parameter of `Cabsl`:
 *
 *     #ifdef __INTELLISENSE__
 *     #define INTELLISENSE_PREFIX Class::
 *     #endif
 *
 * @author Thomas Röfer
 */

#pragma once

#include <unordered_map>
#include "Math/Eigen.h" // Not used, but avoids naming conflicts
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Streaming/FunctionList.h"
#include "Streaming/InStreams.h"
#include "Streaming/OutStreams.h"
#include "Streaming/TypeRegistry.h"

/** Reject Microsoft's traditional preprocessor. */
#if defined _MSC_VER && (!defined _MSVC_TRADITIONAL || _MSVC_TRADITIONAL)
#error "This code requires the standard preprocessor (/Zc:preprocessor)."
#endif

namespace cabsl
{
  /**
   * Base class for helper structures that makes sure that the
   * destructor is virtual.
   */
  struct StructBase
  {
    virtual ~StructBase() = default;
  };

  /** Wrapper for InMapFile. */
  struct InFileStream : public InMapFile
  {
    InFileStream(const std::string& basename) : InMapFile("option" + basename + ".cfg") {ASSERT(exists());}
  };

  /** Wrapper for OutMapMemory. */
  class OutStringStream
  {
    template<typename U> struct isStreamableBase
    {
      template<typename V> static auto test(V*) -> decltype(std::declval<EnumHelpers::Out2&>() << std::declval<V>());
      template<typename> static auto test(...) -> std::false_type;
      using type = typename std::negation<typename std::is_same<std::false_type, decltype(test<U>(nullptr))>::type>::type;
    };
    template<typename U> struct isStreamable : isStreamableBase<U>::type {};
    OutMapMemory stream;

  public:
    OutStringStream() : stream(true, 1024) {}
    std::string str() {std::string text = stream.data(); return text.size() >= 5 ? text.substr(3, text.size() - 5) : text;}
    template<typename T> typename std::enable_if<isStreamable<T>::value, Out&>::type
      operator<<(const T& value) {Streaming::streamIt(stream, "", value); return stream;}
  };

  /**
   * The base class for CABSL behaviors.
   * Note: Private variables that cannot be declared as private start with an
   * underscore.
   * @tparam CabslBehavior_ The class that implements the behavior. It must also be
   * derived from this class.
   * @tparam InFileStream_ A class with an interface compatible to cabsl::InFileStream. It
   * is used to load definitions from configuration files.
   * @tparam OutStringStream_ A class with an interface compatible to std::stringstream.
   * Instances of the class are used to add arguments and variables to the activation
   * graph.
   */
  template<typename CabslBehavior_, typename InFileStream_ = InFileStream, typename OutStringStream_ = OutStringStream>
    class Cabsl
  {
  public:
    using InFileStream = InFileStream_; /**< This type allows to access the stream class by name. */

  protected:
    using CabslBehavior = CabslBehavior_; /**< This type allows to access the derived class by name. */
    using OutStringStream = OutStringStream_; /**< This type allows to access the stream class by name. */

    /**
     * The context stores the current state of an option.
     */
    class OptionContext
    {
    public:
      /** The different types of states (for implementing `initial_state`, `target_state`, and `aborted_state`). */
      enum StateType
      {
        normalState,
        initialState,
        targetState,
        abortedState
      };

      int state; /**< The state currently selected. This is actually the line number in which the state was declared. */
      const char* stateName; /**< The name of the state (for activation graph). */
      unsigned lastFrame = static_cast<unsigned>(-1); /**< The timestamp of the last frame in which this option was executed (except for the initial state when called from `select_option`). */
      unsigned lastSelectFrame = static_cast<unsigned>(-1); /**< The timestamp of the last frame in which this option was executed (in any case). */
      unsigned optionStart; /**< The time when the option started to run (for `option_time`). */
      unsigned stateStart; /**< The time when the current state started to run (for `state_time`). */
      StateType stateType; /**< The type of the current state in this option. */
      StateType subOptionStateType; /**< The type of the state of the last suboption executed (for `action_done` and `action_aborted`). */
      bool addedToGraph; /**< Was this option already added to the activation graph in this frame? */
      bool transitionExecuted; /**< Has a transition already been executed? True after a state change. */
      bool hasCommonTransition; /**< Does this option have a common transition? Is reset when entering the first state. */
      Streamable* defs = nullptr; /**< Option configuration definitions. */
      StructBase* vars = nullptr; /**< Option variables. */

      /** Destructor. */
      ~OptionContext()
      {
        delete defs;
        delete vars;
      }
    };

    /**
     * Instances of this class are passed as a default argument to each option.
     * They maintain the current state of the option.
     */
    class OptionExecution
    {
      /** Helper to determine, whether U is streamable. */
      template<typename U> struct isStreamableBase
      {
        template<typename V> static auto test(V*) -> decltype(std::declval<OutStringStream&>() << std::declval<V>());
        template<typename> static auto test(...) -> std::false_type;
        using type = typename std::negation<typename std::is_same<std::false_type, decltype(test<U>(nullptr))>::type>::type;
      };
      template<typename U> struct isStreamable : isStreamableBase<U>::type {};

      const char* optionName; /**< The name of the option (for activation graph). */
      Cabsl* instance; /**< The object that encapsulates the behavior. */
      bool fromSelect; /**< Option is called from `select_option`. */
      mutable std::vector<std::string> arguments; /**< Argument names and their values. */

    public:
      OptionContext& context; /**< The context of the state. */

      /**
       * The constructor checks, whether the option was active in the previous frame and
       * if not, it switches back to the initial state.
       * @param optionName The name of the option (for activation graph).
       * @param context The context of the state.
       * @param instance The object that encapsulates the behavior.
       */
      OptionExecution(const char* optionName, OptionContext& context, Cabsl* instance, bool fromSelect = false) :
        optionName(optionName), instance(instance), fromSelect(fromSelect), context(context)
      {
        if(context.lastFrame != instance->lastFrameTime && context.lastFrame != instance->_currentFrameTime)
        {
          context.optionStart = instance->_currentFrameTime; // option started now
          context.stateStart = instance->_currentFrameTime; // initial state started now
          context.state = 0; // initial state is always marked with a 0
          context.stateType = OptionContext::initialState;
        }
        if(context.lastSelectFrame != instance->lastFrameTime && context.lastSelectFrame != instance->_currentFrameTime)
          context.subOptionStateType = OptionContext::normalState; // reset `action_done` and `action_aborted`
        context.addedToGraph = false; // not added to graph yet
        context.transitionExecuted = false; // no transition executed yet
        context.hasCommonTransition = false; // until one is found, it is assumed that there is no common transition
        ++instance->depth; // increase depth counter for activation graph
      }

      /**
       * The destructor cleans up the option context.
       */
      ~OptionExecution()
      {
        if(!fromSelect || context.stateType != OptionContext::initialState)
        {
          addToActivationGraph(); // add to activation graph if it has not been already
          context.lastFrame = instance->_currentFrameTime; // Remember that this option was called in this frame
        }
        context.lastSelectFrame = instance->_currentFrameTime; // Remember that this option was called in this frame (even in `select_option`/`initial_state`)
        --instance->depth; // decrease depth counter for activation graph
        context.subOptionStateType = instance->stateType; // remember the state type of the last sub option called
        instance->stateType = context.stateType; // publish the state type of this option, so the caller can grab it
      }

      /**
       * The method is executed whenever the state is changed.
       * @param newState The new state to which it was changed.
       * @param stateType The type of the new state.
       */
      void updateState(int newState, typename OptionContext::StateType stateType) const
      {
        ASSERT(context.hasCommonTransition != context.transitionExecuted); // `[common_]transition` is missing
        context.transitionExecuted = true; // a transition was executed, do not execute another one
        if(context.state != newState) // ignore transitions that stay in the same state
        {
          context.state = newState;
          context.stateStart = instance->_currentFrameTime; // state started now
          context.stateType = stateType; // remember type of this state
        }
      }

      /**
       * Adds a string description containing the current value of an argument to the list of arguments.
       * The description is only added if the argument is streamable.
       * @tparam U The type of the argument.
       * @param value The current value of the argument.
       */
      template<typename U> typename std::enable_if<isStreamable<U>::value>::type addArgument(const char* name, const U& value) const
      {
        name += 1 + static_cast<int>(std::string(name).find_last_of(" )"));
        OutStringStream stream;
        stream << value;
        arguments.emplace_back(name + (" = " + stream.str()));
      }

      /** Does not write the argument to a stream, because it is not streamable. */
      template<typename U> typename std::enable_if<!isStreamable<U>::value>::type addArgument(const char*, const U&) const {}

      /**
       * The method adds information about the current option and state to the activation graph.
       * It suppresses adding it twice in the same frame.
       */
      void addToActivationGraph() const
      {
        if(!context.addedToGraph && instance->activationGraph)
        {
          instance->activationGraph->graph.emplace_back(optionName, instance->depth,
                                                        context.stateName,
                                                        instance->_currentFrameTime - context.optionStart,
                                                        instance->_currentFrameTime - context.stateStart,
                                                        arguments);
          context.addedToGraph = true;
        }
      }
    };

    /** A class to store information about an option. */
    struct OptionDescriptor
    {
      const char* name; /**< The name of the option. */
      void (CabslBehavior::*option)(const OptionExecution&); /**< The option method. */
      size_t offsetOfContext; /**< The memory offset of the context within the behavior class. */
      int index; /**< The index of the option (for the enum of all options). */

      /** Default constructor, because STL types need one. */
      OptionDescriptor() = default;

      /**
       * Constructor.
       * @param name The name of the option.
       * @param option The option method.
       * @param offsetOfContext The memory offset of the context within the behavior class.
       */
      OptionDescriptor(const char* name, void (CabslBehavior::*option)(const OptionExecution&), size_t offsetOfContext) :
        name(name), option(option), offsetOfContext(offsetOfContext), index(0)
      {}
    };

  public:
    /** A class that collects information about all options in the behavior. */
    class OptionInfos
    {
    private:
      static std::unordered_map<std::string, const OptionDescriptor*>* optionsByName; /**< All argumentless options, indexed by their names. */
      static std::vector<void (*)()>* initHandlers; /**< All initialization handlers for options with definitions. */

    public:
      enum Option : unsigned char {none}; /**< A dummy enum for all options. */

      /** The destructor frees the global object. */
      ~OptionInfos()
      {
        delete optionsByName;
        delete initHandlers;
        optionsByName = nullptr;
        initHandlers = nullptr;
      }

      /**
       * The method prepares the collection of information about all options in optionByIndex
       * and optionsByName. It also adds a dummy option descriptor at index 0 with the name "none".
       */
      static void init()
      {
        ASSERT(!optionsByName);
        optionsByName = new std::unordered_map<std::string, const OptionDescriptor*>;
        static OptionDescriptor descriptor("none", 0, 0);
        (*optionsByName)[descriptor.name] = &descriptor;
        TypeRegistry::addEnum(typeid(Option).name());
        TypeRegistry::addEnumConstant(typeid(Option).name(), "none");
      }

      /**
       * The method adds information about an option to the collections.
       * It will be called from the constructors of static objects created for each
       * option.
       * This method is only called for options without arguments, because only they
       * can be called externally.
       * @param descriptor A description of an option.
       */
      static void add(OptionDescriptor& descriptor)
      {
        if(!optionsByName)
          init();

        ASSERT(optionsByName);
        if(optionsByName->find(descriptor.name) == optionsByName->end()) // only register once
        {
          (*optionsByName)[descriptor.name] = &descriptor;
          TypeRegistry::addEnumConstant(typeid(Option).name(), descriptor.name);
        }
      }

      /**
       * The method registers a handler to initialize definitions.
       * @param initHandler The address of the handler.
       */
      static void add(void (*initHandler)())
      {
        if(!initHandlers)
          initHandlers = new std::vector<void (*)()>;
        initHandlers->push_back(initHandler);
      }

      /**
       * The method executes a certain option. Note that only argumentless options can be
       * executed.
       * @param behavior The behavior instance.
       * @param option The name of the option.
       * @param fromSelect Was this method called from `select_option`?
       * @return Was the option actually executed?
       */
      static bool execute(CabslBehavior* behavior, const std::string& option, bool fromSelect = false)
      {
        auto pair = optionsByName->find(option);
        if(pair != optionsByName->end())
        {
          const OptionDescriptor& descriptor = *pair->second;
          OptionContext& context = *reinterpret_cast<OptionContext*>(reinterpret_cast<char*>(behavior) + descriptor.offsetOfContext);
          (behavior->*(descriptor.option))(OptionExecution(descriptor.name, context, behavior, fromSelect));
          return context.stateType != OptionContext::initialState;
        }
        else
          return false;
      }

      /**
       * The method executes a certain option. Note that only argumentless options can be
       * executed.
       * @param behavior The behavior instance.
       * @param option The name of the option.
       * @param fromSelect Was this method called from `select_option`?
       * @return Was the option actually executed?
       */
      static bool execute(CabslBehavior* behavior, Option option, bool fromSelect = false)
      {
        return execute(behavior, TypeRegistry::getEnumName(option), fromSelect);
      }

      /**
       * The method executes a list of options. It stops after the first option that reports that
       * it was actually executed.
       * @param behavior The behavior instance.
       * @param options The list of option names. The options are executed in that order.
       * @return Was an option actually executed?
       */
      template<typename T> static bool execute(CabslBehavior* behavior, const std::vector<T>& options)
      {
        for(const T& option : options)
          if(execute(behavior, option, true))
            return true;
        return false;
      }

      /** Executes all handlers that initialize the definitions. */
      static void executeInitHandlers()
      {
        if(initHandlers)
          for(void (*initHandler)() : *initHandlers)
            initHandler();
      }
    };

  private:
    static OptionInfos collectOptions; /**< This global instantiation collects data about all options. */
    typename OptionContext::StateType stateType = OptionContext::normalState; /**< The state type of the last option called. */
    unsigned lastFrameTime = 0; /**< The timestamp of the last time the behavior was executed. */
    int depth = 0; /**< The depth level of the current option. Used for activation graph. */
    ActivationGraph* activationGraph; /**< The activation graph for debug output. Can be zero if not set. */

  protected:
    static thread_local Cabsl* _theInstance; /**< The instance of this behavior used. */
    unsigned _currentFrameTime = 0; /**< The timestamp of the last time the behavior was executed. */

    /**
     * Constructor.
     * @param activationGraph When set, the activation graph will be filled with the
     *                        options and states executed in each frame.
     */
    Cabsl(ActivationGraph* activationGraph = nullptr) :
      activationGraph(activationGraph)
    {
      static_cast<void>(&collectOptions); // Enforce linking of this global object
    }

  public:
    /**
     * Must be called at the beginning of each behavior execution cycle even if no option is called.
     * @param frameTime The current time in ms.
     */
    void beginFrame(unsigned frameTime)
    {
      if(SystemCall::getMode() == SystemCall::logFileReplay && frameTime < lastFrameTime)
        _currentFrameTime = frameTime;
      else
        _currentFrameTime = std::max(frameTime, lastFrameTime + 1);
      if(activationGraph)
        activationGraph->graph.clear();
      _theInstance = this;
      OptionInfos::executeInitHandlers();
    }

    /**
     * Execute an option as a root.
     * Several root options can be executed in a single behavior execution cycle.
     * @param root The root option that is executed.
     */
    template<typename T> void execute(const T& root)
    {
      OptionInfos::execute(static_cast<CabslBehavior*>(this), root);
    }

    /** Must be called at the end of each behavior execution cycle even if no option is called. */
    void endFrame()
    {
      _theInstance = nullptr;
      lastFrameTime = _currentFrameTime;
      ASSERT(depth == 0);
    }
  };

  template<typename CabslBehavior, typename InFileStream, typename OutStringStream>
    thread_local Cabsl<CabslBehavior, InFileStream, OutStringStream>* Cabsl<CabslBehavior, InFileStream, OutStringStream>::_theInstance;
  template<typename CabslBehavior, typename InFileStream, typename OutStringStream>
    std::unordered_map<std::string, const typename Cabsl<CabslBehavior, InFileStream, OutStringStream>::OptionDescriptor*>* Cabsl<CabslBehavior, InFileStream, OutStringStream>::OptionInfos::optionsByName;
  template<typename CabslBehavior, typename InFileStream, typename OutStringStream>
    std::vector<void (*)()>* Cabsl<CabslBehavior, InFileStream, OutStringStream>::OptionInfos::initHandlers;
  template<typename CabslBehavior, typename InFileStream, typename OutStringStream>
    typename Cabsl<CabslBehavior, InFileStream, OutStringStream>::OptionInfos Cabsl<CabslBehavior, InFileStream, OutStringStream>::collectOptions;

  /**
   * Together with decltype, the following template allows to use any type
   * for declarations, even array types such as int[4]. It also works with
   * template parameters without the use of typename.
   * decltype(TypeWrapper<myType>::type) myVar;
   */
  using Streaming::TypeWrapper;
}

/**
 * The macro defines a state. It must be followed by a block of code that defines the state's body.
 * @param name The name of the state.
 */
#define state(name) _state(name, __LINE__, OptionContext::normalState)

/**
 * The macro defines a target state. It must be followed by a block of code that defines the state's body.
 * A parent option can check whether a target state has been reached through `action_done`.
 * @param name The name of the target state.
 */
#define target_state(name) _state(name, __LINE__, OptionContext::targetState)

/**
 * The macro defines an aborted state. It must be followed by a block of code that defines the state's body.
 * A parent option can check whether an aborted state has been reached through `action_aborted`.
 * @param name The name of the aborted state.
 */
#define aborted_state(name) _state(name, __LINE__, OptionContext::abortedState)

/**
 * The macro defines an option. It must be followed by a block of code that defines the option's body.
 * The option gets an additional argument that manages its context. If the option has arguments,
 * two methods are generated. The first one adds the arguments to the execution environment and calls
 * the second one.
 * @param ... The name of the option and an arbitrary number of arguments. They can include default
 *            arguments at the end. Their syntax is described at the beginning of this file.
 */
#define option(...) _CABSL_OPTION(_CABSL_TUPLE_SIZE(__VA_ARGS__), __VA_ARGS__)

// Determine whether a class, arguments, definitions, and/or variables were specified.
// Then _CABSL_OPTION_II is called with all this information.
#define _CABSL_OPTION(n, ...) _CABSL_JOIN(_CABSL_OPTION_, n) (__VA_ARGS__)
#define _CABSL_OPTION_1(name) _CABSL_OPTION_I(name, , , , , ignore)
#define _CABSL_OPTION_2(name, ...) _CABSL_OPTION_I(name, _CABSL_HAS_ARGS(__VA_ARGS__), _CABSL_HAS_DEFS(__VA_ARGS__), _CABSL_HAS_LOAD(__VA_ARGS__), _CABSL_HAS_VARS(__VA_ARGS__), __VA_ARGS__)
#define _CABSL_OPTION_3(...) _CABSL_OPTION_2(__VA_ARGS__)
#define _CABSL_OPTION_4(...) _CABSL_OPTION_2(__VA_ARGS__)
#define _CABSL_OPTION_I(name, hasArgs, hasDefs, hasLoad, hasVars, ...) _CABSL_JOIN(_CABSL_OPTION_I_, _CABSL_SEQ_SIZE(name))(name, hasArgs, hasDefs, hasLoad, hasVars, __VA_ARGS__)
#define _CABSL_OPTION_I_0(name, hasArgs, hasDefs, hasLoad, hasVars, ...) _CABSL_OPTION_II(name, , , hasArgs, hasDefs, hasLoad, hasVars, __VA_ARGS__)
#define _CABSL_OPTION_I_1(name, hasArgs, hasDefs, hasLoad, hasVars, ...) _CABSL_OPTION_II(_CABSL_VAR(name), _CABSL_OPTION_I_1_I(name), 1, hasArgs, hasDefs, hasLoad, hasVars, __VA_ARGS__)
#define _CABSL_OPTION_I_1_I(name) _CABSL_DECL_I name))

// Generate the actual code for the option header. The `has` parameters are either
// `1` or empty.
#define _CABSL_OPTION_II(name, class, hasClass, hasArgs, hasDefs, hasLoad, hasVars, ...) \
  _CABSL_DECL_CONTEXT_##hasClass##_##hasArgs(name) \
  _CABSL_STRUCT_ARGS_##hasClass##_##hasArgs(name, __VA_ARGS__) \
  _CABSL_NAMESPACE_BEGIN_##hasClass(class) \
  _CABSL_STRUCT_DEFS_##hasClass##_##hasDefs(name, class, __VA_ARGS__) \
  _CABSL_STRUCT_VARS_##hasVars(name, __VA_ARGS__) \
  _CABSL_NAMESPACE_END_##hasClass(class) \
  _CABSL_INIT_DEFS_##hasClass##_##hasDefs##_##hasLoad(name, class) \
  _CABSL_FUNS_##hasClass##_##hasArgs##_##hasDefs##_##hasVars(name, class, __VA_ARGS__)

// Declare the option context if executed in the header file (inline).
// Also generate registration method for the option if it has no arguments.
#define _CABSL_DECL_CONTEXT__(name) \
  OptionContext _##name##Context; \
  static void _##name##DescriptorReg() \
  { \
    PUBLISH(_##name##DescriptorReg); \
    static OptionDescriptor descriptor(#name, reinterpret_cast<void (CabslBehavior::*)(const OptionExecution&)>(&CabslBehavior::name), \
                                       reinterpret_cast<size_t>(&reinterpret_cast<CabslBehavior*>(16)->_##name##Context) - 16); \
    OptionInfos::add(descriptor); \
  }
#define _CABSL_DECL_CONTEXT_1_(name, ...)
#define _CABSL_DECL_CONTEXT__1(name, ...) \
  OptionContext _##name##Context;
#define _CABSL_DECL_CONTEXT_1_1(name, ...)

// Define a structure that contains arguments.
// The structure is only defined if it is needed (second `1` of the name).
// It is only defined if the first `1` is not present, because the structure
// for arguments is only defined inline, never in the implementation file.
#define _CABSL_STRUCT_ARGS__(name, ...)
#define _CABSL_STRUCT_ARGS_1_(name, ...)
#define _CABSL_STRUCT_ARGS__1(name, ...) _CABSL_STRUCT_ARGS_I(name, _CABSL_GET_ARGS(__VA_ARGS__), ignore)
#define _CABSL_STRUCT_ARGS_I(name, ...) _CABSL_STRUCT_ARGS_II(name, _CABSL_TUPLE_SIZE(__VA_ARGS__), __VA_ARGS__)
#define _CABSL_STRUCT_ARGS_II(name, n, ...) _CABSL_STRUCT_ARGS_III(name, n, (_CABSL_STRUCT_WITH_INIT, __VA_ARGS__))
#define _CABSL_STRUCT_ARGS_III(name, n, pair) \
  struct _##name##Args \
  { \
    _STREAM_ATTR_##n pair \
  };
#define _CABSL_STRUCT_ARGS_1_1(name, ...)

// Generate the declaration and optional initialization of a field in the structure.
#define _CABSL_STRUCT_WITH_INIT(seq) std::remove_const<std::remove_reference<decltype(cabsl::TypeWrapper<_CABSL_DECL_I seq))>::type)>::type>::type _CABSL_VAR(seq) _CABSL_INIT(seq);

// Define a structure for definitions. They are streamable, so the STREAMABLE macro is used.
#define _CABSL_STRUCT_DEFS__(name, class, ...)
#define _CABSL_STRUCT_DEFS_1_(name, class, ...)
#define _CABSL_STRUCT_DEFS__1(name, class, ...) _CABSL_STRUCT_DEFS_I(name, , _CABSL_GET_DEFS(__VA_ARGS__))
#define _CABSL_STRUCT_DEFS_1_1(name, class, ...) _CABSL_STRUCT_DEFS_I(name, class::, _CABSL_GET_DEFS(__VA_ARGS__))
#define _CABSL_STRUCT_DEFS_I(name, prefix, ...) \
  STREAMABLE(_##name##Defs, \
  { \
     void _read(prefix InFileStream& stream) \
     {\
       stream >> *this; \
     },\
     __VA_ARGS__, \
  });

// Define a structure that contains variables.
// The structure is only defined if it is needed (addition `1` of the name).
#define _CABSL_STRUCT_VARS_(name, ...)
#define _CABSL_STRUCT_VARS_1(name, ...) _CABSL_STRUCT_VARS_I(name, _CABSL_GET_VARS(__VA_ARGS__), ignore)
#define _CABSL_STRUCT_VARS_I(name, ...) _CABSL_STRUCT_VARS_II(name, _CABSL_TUPLE_SIZE(__VA_ARGS__), __VA_ARGS__)
#define _CABSL_STRUCT_VARS_II(name, n, ...) _CABSL_STRUCT_VARS_III(name, n, (_CABSL_STRUCT_WITHOUT_INIT, __VA_ARGS__))
#define _CABSL_STRUCT_VARS_III(name, n, pair) \
  struct _##name##Vars : public cabsl::StructBase \
  { \
    _STREAM_ATTR_##n pair \
  };

// Generate the declaration of a field in the structure.
#define _CABSL_STRUCT_WITHOUT_INIT(seq) std::remove_const<std::remove_reference<decltype(cabsl::TypeWrapper<_CABSL_DECL_I seq))>::type)>::type>::type _CABSL_VAR(seq);

// Define an initialization handler and a function that registers it.
// It is distinguished whether a class was specified (implementation file) or not (header),
// whether there actually are definitions, and whether they are read from a file.
#define _CABSL_INIT_DEFS___(name, class) \
  static void _##name##Init(); \
  static void _##name##InitReg();
#define _CABSL_INIT_DEFS_1__(name, class)
#define _CABSL_INIT_DEFS__1_(name, class) _CABSL_INIT_DEFS_I(name, , static, )
#define _CABSL_INIT_DEFS__1_1(name, class) _CABSL_INIT_DEFS_I(name, , static, InFileStream _stream(#name); _defs->_read(_stream);)
#define _CABSL_INIT_DEFS_1_1_(name, class) _CABSL_INIT_DEFS_I(name, class::, , )
#define _CABSL_INIT_DEFS_1_1_1(name, class) _CABSL_INIT_DEFS_I(name, class::, , InFileStream _stream(#name); _defs->_read(_stream);)
#define _CABSL_INIT_DEFS_I(name, class, prefix, load) \
  prefix void class _##name##Init() \
  { \
    _##name##Defs*& _defs = reinterpret_cast<_##name##Defs*&>(static_cast<CabslBehavior*>(_theInstance)->_##name##Context.defs); \
    if(!_defs) \
    { \
      _defs = new _##name##Defs(); \
      load \
    } \
    MODIFY("option:" #name, *_defs); \
  } \
  prefix void class _##name##InitReg() \
  { \
    PUBLISH(_##name##InitReg); \
    OptionInfos::add(&CabslBehavior::_##name##Init); \
  }

// Generate start of namespace if used in implementation file.
#define _CABSL_NAMESPACE_BEGIN_(class)
#define _CABSL_NAMESPACE_BEGIN_1(class) namespace _ns##class {

// Generate end of namespace if used in implementation file. Import the namespace, so
// some other macros do not need to distinguish between the namespace/no namespace cases.
#define _CABSL_NAMESPACE_END_(class)
#define _CABSL_NAMESPACE_END_1(class) } using namespace _ns##class;

// Generate 16 different versions based on the parameters specified:
// - Class name defined (not inline, i.e. in implementation file) or not (inline in class body)
// - Arguments defined or not
// - Definitions defined or not
// - Variables defined or not

// Inline, no `args`, no `defs`, no `vars`
#define _CABSL_FUNS____(name, class, ...) \
  _CABSL_NOARGS_HEAD(name)

// Not inline, no `args`, no `defs`, no `vars`
// Option is called directly. Default argument was declared in header.
#define _CABSL_FUNS_1___(name, class, ...) \
  void class::name(const OptionExecution& _o)

// Inline, `args`, no `defs`, no `vars`
// Helper needed to stream and translate arguments.
#define _CABSL_FUNS__1__(name, class, ...) \
  _CABSL_ARGS_HEAD(name, __VA_ARGS__) \
    _##name(_CABSL_APPLY(_CABSL_PASS_ARG, _CABSL_GET_ARGS(__VA_ARGS__)) _o); \
  } \
  void _##name(_CABSL_APPLY(_CABSL_DECL_ARG, _CABSL_GET_ARGS(__VA_ARGS__)) const OptionExecution& _o)

// Not inline, `args`, no `defs`, no `vars`
// Helper was declared in header that calls actual option.
// There should be no defaults for arguments. Generate them if they are, so the compiler will complain.
#define _CABSL_FUNS_1_1__(name, class, ...) \
  void class::_##name(_CABSL_APPLY(_CABSL_DECL_ARG_WITH_INIT, _CABSL_GET_ARGS(__VA_ARGS__)) const OptionExecution& _o)

// Inline, no `args`, `defs`, no `vars`
// Helper needed to handle `defs`.
#define _CABSL_FUNS___1_(name, class, ...) \
  _CABSL_NOARGS_HEAD(name) \
  { \
    _CABSL_DEFS_IMPL(name) \
    _##name(_CABSL_APPLY(_CABSL_PASS_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _o); \
  } \
  void _##name(_CABSL_APPLY(_CABSL_DECL_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) const OptionExecution& _o)

// Not inline, no `args`, `defs`, no `vars`
// Helper needed to handle `defs`. Wrapper class needed to define another method.
#define _CABSL_FUNS_1__1_(name, class, ...) \
  namespace _ns##class \
  { \
    struct name##Wrapper : public class \
    { \
      void name(_CABSL_APPLY(_CABSL_DECL_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) const OptionExecution& _o); \
    }; \
  } \
  void class::name(const OptionExecution& _o) \
  { \
    _CABSL_DEFS_IMPL(name) \
    reinterpret_cast<_ns##class::name##Wrapper*>(this)->name(_CABSL_APPLY(_CABSL_PASS_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _o); \
  } \
  void _ns##class::name##Wrapper::name(_CABSL_APPLY(_CABSL_DECL_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) const OptionExecution& _o)

// Inline, `args`, `defs`, no `vars`
// Helper needed to stream and translate arguments and handle `defs`.
#define _CABSL_FUNS__1_1_(name, class, ...) \
  _CABSL_ARGS_HEAD(name, __VA_ARGS__) \
    _CABSL_DEFS_IMPL(name) \
    _##name(_CABSL_APPLY(_CABSL_PASS_ARG, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_PASS_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _o); \
  } \
  void _##name(_CABSL_APPLY(_CABSL_DECL_ARG, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) const OptionExecution& _o)

// Not inline, `args`, `defs`, no `vars`
// Header already handled `args`. Second helper needed to handle `defs`.
// Wrapper class needed to define a third method.
#define _CABSL_FUNS_1_1_1_(name, class, ...) \
  namespace _ns##class \
  { \
    struct name##Wrapper : public class \
    { \
      void name(_CABSL_APPLY(_CABSL_DECL_ARG_WITH_INIT, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) const OptionExecution& _o); \
    }; \
  } \
  void class::_##name(_CABSL_APPLY(_CABSL_DECL_ARG_WITH_INIT, _CABSL_GET_ARGS(__VA_ARGS__)) const OptionExecution& _o) \
  { \
    _CABSL_DEFS_IMPL(name) \
    reinterpret_cast<_ns##class::name##Wrapper*>(this)->name(_CABSL_APPLY(_CABSL_PASS_PARAM, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_PASS_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _o); \
  } \
  void _ns##class::name##Wrapper::name(_CABSL_APPLY(_CABSL_DECL_ARG_WITH_INIT, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) const OptionExecution& _o)

// Inline, no `args`, no `defs`, `vars`
// Helper needed to handle `vars`.
#define _CABSL_FUNS____1(name, class, ...) \
  _CABSL_NOARGS_HEAD(name) \
  { \
    _CABSL_VARS_IMPL(name, __VA_ARGS__) \
    _##name(_CABSL_APPLY(_CABSL_PASS_VAR, _CABSL_GET_VARS(__VA_ARGS__)) _o); \
  } \
  void _##name(_CABSL_APPLY(_CABSL_DECL_VAR, _CABSL_GET_VARS(__VA_ARGS__)) const OptionExecution& _o)

// Not inline, no `args`, no `defs`, `vars`
// Helper needed to handle vars. Wrapper class needed to define another method.
#define _CABSL_FUNS_1___1(name, class, ...) \
  namespace _ns##class \
  { \
    struct name##Wrapper : public class \
    { \
      void name(_CABSL_APPLY(_CABSL_DECL_VAR, _CABSL_GET_VARS(__VA_ARGS__)) const OptionExecution& _o); \
    }; \
  } \
  void class::name(const OptionExecution& _o) \
  { \
    _CABSL_VARS_IMPL(name, __VA_ARGS__) \
    reinterpret_cast<_ns##class::name##Wrapper*>(this)->name(_CABSL_APPLY(_CABSL_PASS_VAR, _CABSL_GET_VARS(__VA_ARGS__)) _o); \
  } \
  void _ns##class::name##Wrapper::name(_CABSL_APPLY(_CABSL_DECL_VAR, _CABSL_GET_VARS(__VA_ARGS__)) const OptionExecution& _o)

// Inline, `args`, no `defs`, `vars`
// Helper needed to stream and translate arguments and handle `vars`.
#define _CABSL_FUNS__1__1(name, class, ...) \
  _CABSL_ARGS_HEAD(name, __VA_ARGS__) \
    _CABSL_VARS_IMPL(name, __VA_ARGS__) \
    _##name(_CABSL_APPLY(_CABSL_PASS_ARG, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_PASS_VAR, _CABSL_GET_VARS(__VA_ARGS__)) _o); \
  } \
  void _##name(_CABSL_APPLY(_CABSL_DECL_ARG, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_VAR, _CABSL_GET_VARS(__VA_ARGS__)) const OptionExecution& _o)

// Not inline, `args`, no `defs`, `vars`
// Header already handled `args`. Second helper needed to handle `vars`.
// Wrapper class needed to define a third method.
#define _CABSL_FUNS_1_1__1(name, class, ...) \
  namespace _ns##class \
  { \
    struct name##Wrapper : public class \
    { \
      void name(_CABSL_APPLY(_CABSL_DECL_ARG_WITH_INIT, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_VAR, _CABSL_GET_VARS(__VA_ARGS__)) const OptionExecution& _o); \
    }; \
  } \
  void class::_##name(_CABSL_APPLY(_CABSL_DECL_ARG_WITH_INIT, _CABSL_GET_ARGS(__VA_ARGS__)) const OptionExecution& _o) \
  { \
    _CABSL_VARS_IMPL(name, __VA_ARGS__) \
    reinterpret_cast<_ns##class::name##Wrapper*>(this)->name(_CABSL_APPLY(_CABSL_PASS_PARAM, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_PASS_VAR, _CABSL_GET_VARS(__VA_ARGS__)) _o); \
  } \
  void _ns##class::name##Wrapper::name(_CABSL_APPLY(_CABSL_DECL_ARG_WITH_INIT, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_VAR, _CABSL_GET_VARS(__VA_ARGS__)) const OptionExecution& _o)

// Inline, no `args`, `defs`, `vars`
// Helper needed to handle `defs` and `vars`.
#define _CABSL_FUNS___1_1(name, class, ...) \
  _CABSL_NOARGS_HEAD(name) \
  { \
    _CABSL_DEFS_IMPL(name) \
    _CABSL_VARS_IMPL(name, __VA_ARGS__) \
    _##name(_CABSL_APPLY(_CABSL_PASS_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_PASS_VAR, _CABSL_GET_VARS(__VA_ARGS__)) _o); \
  } \
  void _##name(_CABSL_APPLY(_CABSL_DECL_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_VAR, _CABSL_GET_VARS(__VA_ARGS__)) const OptionExecution& _o)

// Not inline, no `args`, `defs`, `vars`
// Helper needed to handle `defs` and `vars`.
// Wrapper class needed to define another method.
#define _CABSL_FUNS_1__1_1(name, class, ...) \
  namespace _ns##class \
  { \
    struct name##Wrapper : public class \
    { \
      void name(_CABSL_APPLY(_CABSL_DECL_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_VAR, _CABSL_GET_VARS(__VA_ARGS__)) const OptionExecution& _o); \
    }; \
  } \
  void class::name(const OptionExecution& _o) \
  { \
    _CABSL_DEFS_IMPL(name) \
    _CABSL_VARS_IMPL(name, __VA_ARGS__) \
    reinterpret_cast<_ns##class::name##Wrapper*>(this)->name(_CABSL_APPLY(_CABSL_PASS_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_PASS_VAR, _CABSL_GET_VARS(__VA_ARGS__)) _o); \
  } \
  void _ns##class::name##Wrapper::name(_CABSL_APPLY(_CABSL_DECL_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_VAR, _CABSL_GET_VARS(__VA_ARGS__)) const OptionExecution& _o)

// Inline, `args`, `defs`, `vars`
// Helper needed to stream and translate arguments and handle `defs` and `vars`.
#define _CABSL_FUNS__1_1_1(name, class, ...) \
  _CABSL_ARGS_HEAD(name, __VA_ARGS__) \
    _CABSL_DEFS_IMPL(name) \
    _CABSL_VARS_IMPL(name, __VA_ARGS__) \
    _##name(_CABSL_APPLY(_CABSL_PASS_ARG, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_PASS_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_PASS_VAR, _CABSL_GET_VARS(__VA_ARGS__)) _o); \
  } \
  void _##name(_CABSL_APPLY(_CABSL_DECL_ARG, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_VAR, _CABSL_GET_VARS(__VA_ARGS__)) const OptionExecution& _o)

// Not inline, `args`, `defs`, `vars`
// Header already handled `args`. Second helper needed to handle `defs` and `vars`.
// Wrapper class needed to define a third method.
#define _CABSL_FUNS_1_1_1_1(name, class, ...) \
  namespace _ns##class \
  { \
    struct name##Wrapper : public class \
    { \
      void name(_CABSL_APPLY(_CABSL_DECL_ARG_WITH_INIT, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_VAR, _CABSL_GET_VARS(__VA_ARGS__)) const OptionExecution& _o); \
    }; \
  } \
  void class::_##name(_CABSL_APPLY(_CABSL_DECL_ARG_WITH_INIT, _CABSL_GET_ARGS(__VA_ARGS__)) const OptionExecution& _o) \
  { \
    _CABSL_DEFS_IMPL(name) \
    _CABSL_VARS_IMPL(name, __VA_ARGS__) \
    reinterpret_cast<_ns##class::name##Wrapper*>(this)->name(_CABSL_APPLY(_CABSL_PASS_PARAM, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_PASS_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_PASS_VAR, _CABSL_GET_VARS(__VA_ARGS__)) _o); \
  } \
  void _ns##class::name##Wrapper::name(_CABSL_APPLY(_CABSL_DECL_ARG_WITH_INIT, _CABSL_GET_ARGS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_DEF, _CABSL_GET_DEFS(__VA_ARGS__)) _CABSL_APPLY(_CABSL_DECL_VAR, _CABSL_GET_VARS(__VA_ARGS__)) const OptionExecution& _o)

// Method head for option without arguments.
#define _CABSL_NOARGS_HEAD(name) \
  void name(const OptionExecution& _o = OptionExecution(#name, static_cast<CabslBehavior*>(_theInstance)->_##name##Context, _theInstance))

// Method head for option with arguments. Also streams the arguments.
#define _CABSL_ARGS_HEAD(name, ...) \
  template<typename U = _##name##Args> typename std::enable_if<std::is_default_constructible<U>::value>::type \
  name(const OptionExecution& _o = OptionExecution(#name, static_cast<CabslBehavior*>(_theInstance)->_##name##Context, _theInstance)) \
  { \
    name(_##name##Args(), _o); \
  } \
  void name(const _##name##Args& _args, const OptionExecution& _o = OptionExecution(#name, static_cast<CabslBehavior*>(_theInstance)->_##name##Context, _theInstance)) \
  { \
    _CABSL_APPLY(_CABSL_STREAM_ARG, _CABSL_GET_ARGS(__VA_ARGS__))

// Implementation for definitions.
#define _CABSL_DEFS_IMPL(name) \
  _##name##Defs* _defs = reinterpret_cast<_##name##Defs*>(_o.context.defs);

// Implementation for option variables. If they do not exist yet, they are allocated.
// In the initial state, they are reset. They are also streamed.
#define _CABSL_VARS_IMPL(name, ...) \
  _##name##Vars*& _vars = reinterpret_cast<_##name##Vars*&>(_o.context.vars); \
  if(!_vars) \
    _vars = new _##name##Vars(); \
  if(_o.context.stateType == OptionContext::initialState && !option_time) \
  { \
    _CABSL_APPLY(_CABSL_INIT_VAR, _CABSL_GET_VARS(__VA_ARGS__)) \
  } \
  _CABSL_APPLY(_CABSL_STREAM_VAR, _CABSL_GET_VARS(__VA_ARGS__)) \

// Assign a value to a variable.
#define _CABSL_INIT_VAR(seq) _vars->_CABSL_VAR(seq) = _CABSL_INIT_I_2_I(seq);

// Apply a macro to all values in a list.
#define _CABSL_APPLY(macro, ...) _CABSL_APPLY_I(macro, _CABSL_TUPLE_SIZE(__VA_ARGS__, ignore), __VA_ARGS__)
#define _CABSL_APPLY_I(macro, n, ...) _CABSL_APPLY_II(n, (macro, __VA_ARGS__, ignore))
#define _CABSL_APPLY_II(n, pair) _STREAM_ATTR_##n pair

// Generate an initialization if the declaration contains one.
#define _CABSL_INIT(seq) _CABSL_JOIN(_CABSL_INIT_I_, _CABSL_SEQ_SIZE(seq))(seq)
#define _CABSL_INIT_I_1(...)
#define _CABSL_INIT_I_2(...) = _CABSL_INIT_I_2_I(__VA_ARGS__)

// Generate code for streaming an argument and adding it to the arguments stored in the execution environment.
#define _CABSL_STREAM_ARG(seq) \
  _CABSL_JOIN(_CABSL_STREAM_ARG_, _CABSL_SEQ_SIZE(seq))(seq) \
    _o.addArgument(#seq, _args._CABSL_VAR(seq));

// If a default value exists, only stream arguments that are different from it.
#define _CABSL_STREAM_ARG_1(seq)
#define _CABSL_STREAM_ARG_2(seq) if(const decltype(_args._CABSL_VAR(seq))& _p = _CABSL_INIT_I_2_I(seq); \
    !(_args._CABSL_VAR(seq) == _p))

// Generate an argument declaration for the formal arguments of a method.
#define _CABSL_DECL_ARG(seq) decltype(cabsl::TypeWrapper<_CABSL_DECL_I seq))>::type) _CABSL_VAR(seq),

// Generate an argument declaration with initialization for the formal arguments of a method.
#define _CABSL_DECL_ARG_WITH_INIT(seq) decltype(cabsl::TypeWrapper<_CABSL_DECL_I seq))>::type) _CABSL_VAR(seq) _CABSL_INIT(seq),

// Generate a variable name for the list of actual arguments of a method call.
#define _CABSL_PASS_ARG(seq) _args._CABSL_VAR(seq),

// Directly forward an argument.
#define _CABSL_PASS_PARAM(seq) _CABSL_VAR(seq),

// Generate an argument declaration for the formal arguments of a method.
#define _CABSL_DECL_DEF(seq) const decltype(cabsl::TypeWrapper<_CABSL_DECL_I seq))>::type)& _CABSL_VAR(seq),

// Generate a variable name for the list of actual arguments of a method call.
#define _CABSL_PASS_DEF(seq) _defs->_CABSL_VAR(seq),

// Generate code for streaming a variable and adding it to the arguments stored in the execution environment.
#define _CABSL_STREAM_VAR(seq) \
  _o.addArgument(#seq, _vars->_CABSL_VAR(seq));

// Generate an argument declaration for the formal arguments of a method.
#define _CABSL_DECL_VAR(seq) decltype(cabsl::TypeWrapper<_CABSL_DECL_I seq))>::type)& _CABSL_VAR(seq),

// Generate a variable name for the list of actual arguments of a method call.
#define _CABSL_PASS_VAR(seq) _vars->_CABSL_VAR(seq),

// Does a list contain an `args()` parameter? Empty or `1`.
#define _CABSL_HAS_ARGS(...) _CABSL_HAS_ARGS_I(_CABSL_TUPLE_SIZE(__VA_ARGS__, ignore), __VA_ARGS__, ignore)
#define _CABSL_HAS_ARGS_I(n, ...) _CABSL_HAS_ARGS_II(n, (_CABSL_HAS_ARGS_III, __VA_ARGS__))
#define _CABSL_HAS_ARGS_II(n, pair, ...) _STREAM_ATTR_##n pair
#define _CABSL_HAS_ARGS_III(entry) _CABSL_JOIN(_CABSL_HAS_ARGS_III_, entry)
#define _CABSL_HAS_ARGS_III_args(...) 1
#define _CABSL_HAS_ARGS_III_defs(...)
#define _CABSL_HAS_ARGS_III_load(...)
#define _CABSL_HAS_ARGS_III_vars(...)

// Return the contents of the `args()` parameter in a list.
#define _CABSL_GET_ARGS(...) _CABSL_GET_ARGS_I(_CABSL_TUPLE_SIZE(__VA_ARGS__, ignore), __VA_ARGS__, ignore)
#define _CABSL_GET_ARGS_I(n, ...) _CABSL_GET_ARGS_II(n, (_CABSL_GET_ARGS_III, __VA_ARGS__))
#define _CABSL_GET_ARGS_II(n, pair, ...) _STREAM_ATTR_##n pair
#define _CABSL_GET_ARGS_III(entry) _CABSL_JOIN(_CABSL_GET_ARGS_III_, entry)
#define _CABSL_GET_ARGS_III_args(...) __VA_ARGS__
#define _CABSL_GET_ARGS_III_defs(...)
#define _CABSL_GET_ARGS_III_load(...)
#define _CABSL_GET_ARGS_III_vars(...)

// Does a list contain a `defs()` or `load()` parameter? Empty or `1`.
#define _CABSL_HAS_DEFS(...) _CABSL_HAS_DEFS_I(_CABSL_TUPLE_SIZE(__VA_ARGS__, ignore), __VA_ARGS__, ignore)
#define _CABSL_HAS_DEFS_I(n, ...) _CABSL_HAS_DEFS_II(n, (_CABSL_HAS_DEFS_III, __VA_ARGS__))
#define _CABSL_HAS_DEFS_II(n, pair, ...) _STREAM_ATTR_##n pair
#define _CABSL_HAS_DEFS_III(entry) _CABSL_JOIN(_CABSL_HAS_DEFS_III_, entry)
#define _CABSL_HAS_DEFS_III_args(...)
#define _CABSL_HAS_DEFS_III_defs(...) 1
#define _CABSL_HAS_DEFS_III_load(...) 1
#define _CABSL_HAS_DEFS_III_vars(...)

// Does a list contain a `load()` parameter? Empty or `1`.
#define _CABSL_HAS_LOAD(...) _CABSL_HAS_LOAD_I(_CABSL_TUPLE_SIZE(__VA_ARGS__, ignore), __VA_ARGS__, ignore)
#define _CABSL_HAS_LOAD_I(n, ...) _CABSL_HAS_LOAD_II(n, (_CABSL_HAS_LOAD_III, __VA_ARGS__))
#define _CABSL_HAS_LOAD_II(n, pair, ...) _STREAM_ATTR_##n pair
#define _CABSL_HAS_LOAD_III(entry) _CABSL_JOIN(_CABSL_HAS_LOAD_III_, entry)
#define _CABSL_HAS_LOAD_III_args(...)
#define _CABSL_HAS_LOAD_III_defs(...)
#define _CABSL_HAS_LOAD_III_load(...) 1
#define _CABSL_HAS_LOAD_III_vars(...)

// Return the contents of the `defs()` or `load()` parameter in a list.
#define _CABSL_GET_DEFS(...) _CABSL_GET_DEFS_I(_CABSL_TUPLE_SIZE(__VA_ARGS__, ignore), __VA_ARGS__, ignore)
#define _CABSL_GET_DEFS_I(n, ...) _CABSL_GET_DEFS_II(n, (_CABSL_GET_DEFS_III, __VA_ARGS__))
#define _CABSL_GET_DEFS_II(n, pair, ...) _STREAM_ATTR_##n pair
#define _CABSL_GET_DEFS_III(entry) _CABSL_JOIN(_CABSL_GET_DEFS_III_, entry)
#define _CABSL_GET_DEFS_III_args(...)
#define _CABSL_GET_DEFS_III_defs(...) __VA_ARGS__
#define _CABSL_GET_DEFS_III_load(...) __VA_ARGS__
#define _CABSL_GET_DEFS_III_vars(...)

// Does a list contain a `vars()` parameter? Empty or `1`.
#define _CABSL_HAS_VARS(...) _CABSL_HAS_VARS_I(_CABSL_TUPLE_SIZE(__VA_ARGS__, ignore), __VA_ARGS__, ignore)
#define _CABSL_HAS_VARS_I(n, ...) _CABSL_HAS_VARS_II(n, (_CABSL_HAS_VARS_III, __VA_ARGS__))
#define _CABSL_HAS_VARS_II(n, pair, ...) _STREAM_ATTR_##n pair
#define _CABSL_HAS_VARS_III(entry) _CABSL_JOIN(_CABSL_HAS_VARS_III_, entry)
#define _CABSL_HAS_VARS_III_args(...)
#define _CABSL_HAS_VARS_III_defs(...)
#define _CABSL_HAS_VARS_III_load(...)
#define _CABSL_HAS_VARS_III_vars(...) 1

// Return the contents of the `vars()` parameter in a list.
#define _CABSL_GET_VARS(...) _CABSL_GET_VARS_I(_CABSL_TUPLE_SIZE(__VA_ARGS__, ignore), __VA_ARGS__, ignore)
#define _CABSL_GET_VARS_I(n, ...) _CABSL_GET_VARS_II(n, (_CABSL_GET_VARS_III, __VA_ARGS__))
#define _CABSL_GET_VARS_II(n, pair, ...) _STREAM_ATTR_##n pair
#define _CABSL_GET_VARS_III(entry) _CABSL_JOIN(_CABSL_GET_VARS_III_, entry)
#define _CABSL_GET_VARS_III_args(...)
#define _CABSL_GET_VARS_III_defs(...)
#define _CABSL_GET_VARS_III_load(...)
#define _CABSL_GET_VARS_III_vars(...) __VA_ARGS__

#ifndef __INTELLISENSE__

/**
 * The macro defines an initial state. It must be followed by a block of code that defines the state's body.
 * Since there does not need to be a transition to an initial state, an unreachable goto statement is defined
 * to avoid warnings about unused labels. The initial state also has an unused extra label that simply ensures
 * that whenever there are states, there must be exactly one initial state.
 * @param name The name of the initial state.
 */
#define initial_state(name) \
  initial_state: \
  if(_o.context.state == -1) \
    goto name; \
  _state(name, 0, OptionContext::initialState)

/**
 * The actual code generated for the state macros above.
 * An unreachable goto statement ensures that there is an initial state.
 * @param name The name of the state.
 * @param line The line number in which the state is defined of 0 for the initial state.
 * @param stateType The type of the state.
 */
#define _state(name, line, stateType) \
  if(false) \
  { \
    goto initial_state; \
  name: _o.updateState(line, stateType); \
  } \
  _o.context.hasCommonTransition = false; \
  if(_o.context.state == line && (_o.context.stateName = #name) && (BH_TRACE, true))

/**
 * The macro marks a common transition. It sets a flag so that a transition is accepted,
 * even if not executed through the keyword `transition`.
 */
#define common_transition \
  _o.context.hasCommonTransition = true;

/**
 * The macro marks a transition. It should be followed by a block of code that contains the decision
 * tree that implements the transition.
 * The macro prevents the option from executing more than one transition per frame.
 * Setting the flag here also allows the transition target to check whether actually
 * a transition block was used to jump to the new state.
 */
#define transition \
  if((_o.context.transitionExecuted ^= true))

/**
 * The macro marks an action. It should be followed by a block of code that contains the
 * implementation of the action.
 * The macro adds information about the current option and state to the activation graph,
 * so that it is added before any suboptions called in the action block can add theirs.
 */
#define action _o.addToActivationGraph();

/** The time since the execution of this option started. */
#define option_time int(_currentFrameTime - _o.context.optionStart)

/** The time since the execution of the current state started. */
#define state_time int(_currentFrameTime - _o.context.stateStart)

/** Did a suboption called reached a target state? */
#define action_done (_o.context.subOptionStateType == OptionContext::targetState)

/** Did a suboption called reached an aborted state? */
#define action_aborted (_o.context.subOptionStateType == OptionContext::abortedState)

/**
 * Executes the first applicable option from a list.
 * @param ... The list of options as a `std::vector<std::string/Option>`.
 * @return Was an option executed?
 */
#define select_option(...) OptionInfos::execute(this, __VA_ARGS__)

#else // __INTELLISENSE__

#ifndef INTELLISENSE_PREFIX
#define INTELLISENSE_PREFIX
#endif
#define _CABSL_OPTION_0_0(name) void INTELLISENSE_PREFIX name(const OptionExecution&)
#define _CABSL_OPTION_III(n, name, ...) _CABSL_OPTION_IV(n, name, (_CABSL_STRUCT_WITH_INIT, __VA_ARGS__), (_CABSL_ARG_WITH_INIT, __VA_ARGS__))
#define _CABSL_OPTION_IV(n, name, params1, params2) \
  struct INTELLISENSE_PREFIX _##name##Args \
  { \
    _STREAM_ATTR_##n params1 \
  }; \
  void INTELLISENSE_PREFIX name(const _##name##Args& args, const OptionExecution&) {} \
  void INTELLISENSE_PREFIX _##name(_STREAM_ATTR_##n params2 const OptionExecution&)
#define _CABSL_IOPTION_IV(n, class, name, params1) \
  void _CABSL_DECL_I class))::_##name(_STREAM_ATTR_##n params1 const OptionExecution&)

#define initial_state(name) \
  initial_state: \
  if(false) \
    goto name; \
  _state(name, 0, )

#define _state(name, line, stateType) \
  if(false) \
  { \
    goto initial_state; \
  name:; \
  } \
  if(true)

#define common_transition ;
#define transition if(true)
#define action ;
#define option_time 0
#define state_time 0
#define action_done false
#define action_aborted false
#define select_option(...) false

#endif

// Map `_CABSL` macros to `_STREAM macros`, except for `_STREAM_ATTR_*`.

/** Determine the number of entries in a tuple. */
#define _CABSL_TUPLE_SIZE(...) _STREAM_TUPLE_SIZE(__VA_ARGS__)

/**
 * Determine whether a sequence is of the form `(a) b` or `(a)(b) c`.
 * In the first case, 1 is returned, otherwise 2.
 */
#define _CABSL_SEQ_SIZE(...) _STREAM_SEQ_SIZE(__VA_ARGS__)

/** Simply drop all parameters passed. */
#define _CABSL_DROP(...) _STREAM_DROP(__VA_ARGS__)

/** Concatenate the two parameters. Works in some situations. */
#define _CABSL_CAT(...) _STREAM_CAT(__VA_ARGS__)

/** Concatenate the two parameters. Works in other situations. */
#define _CABSL_JOIN(a, b) _STREAM_JOIN(a, b)

/** Generate the actual declaration. */
#define _CABSL_DECL_I(...) _STREAM_DECL_I(__VA_ARGS__)

/** Extract the variable from the declaration. */
#define _CABSL_VAR(...) _STREAM_VAR(__VA_ARGS__)

/** Generate the initialization code from the declaration if required. */
#define _CABSL_INIT_I_2_I(...) _STREAM_INIT_I_2_I(__VA_ARGS__)
