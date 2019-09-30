/**
 * @file Cabsl.h
 *
 * The file defines a set of macros that define the C-based agent
 * behavior specification language (CABSL). Semantically, it follows
 * the ideas of XABSL.
 *
 * Grammar:
 *
 * <cabsl> ::= { <option> }
 *
 * <option> ::= option '(' <C-ident> { ',' <decl> } ')'
 *              '{'
 *              [ common_transition <transition> ]
 *              { <other_state> } initial_state <state> { <other_state> }
 *              '}'
 *
 * <other_state> ::= ( state | target_state | aborted_state ) <state>
 *
 * <state> ::= '(' C-ident ')'
 *             '{'
 *             [ transition <transition> ]
 *             [ action <action> ]
 *             '}'
 *
 * <transition> ::= '{' <C-ifelse> '}'
 *
 * <action> ::= '{' <C-statements> '}'
 *
 * <C-ifelse> should contain 'goto' statements (names of states are
 * labels). Conditions can use the pre-defined symbols 'state_time',
 * 'option_time', 'action_done', and 'action_aborted'. Within a state, the
 * action <C-statements> can contain calls to other (sub)options.
 * 'action_done' determines whether the last suboption called reached a
 * target state in the previous execution cycle. 'action_aborted' does the
 * same for an aborted state.
 *
 * If Microsoft Visual Studio is used and options are included from separate
 * files, the following preprocessor code might be added before including
 * this file. "Class" has to be replaced by the template parameter of Cabsl:
 *
 * #ifdef __INTELLISENSE__
 * #define INTELLISENSE_PREFIX Class::
 * #endif
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Math/Eigen.h" // Not used, but avoids naming conflicts
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Platform/BHAssert.h"
#include "Tools/FunctionList.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/TypeRegistry.h"
#include <unordered_map>
#include <vector>

/**
 * The base class for CABSL behaviors.
 * Note: Private variables that cannot be declared as private start with an
 * underscore.
 * @tparam T The class that implements the behavior. It must also be derived from
 * this class.
 */
template<typename T> class Cabsl
{
private:
  static const size_t maxNumOfOptions = 500; /**< The maximum number of options. Increase if necessary. */

protected:
  using CabslBehavior = T; /**< This type allows to access the derived class by name. */

  /**
   * The context stores the current state of an option.
   */
  class OptionContext
  {
  public:
    /** The different types of states (for implementing target_state and aborted_state). */
    enum StateType
    {
      normalState,
      targetState,
      abortedState
    };

    int state; /**< The state currently selected. This is actually the line number in which the state was declared. */
    const char* stateName; /**< The name of the state (for activation graph). */
    unsigned lastFrame = 1; /**< The timestamp of the last frame in which this option was executed. */
    unsigned optionStart; /**< The time when the option started to run (for option_time). */
    unsigned stateStart; /**< The time when the current state started to run (for state_time). */
    StateType stateType; /**< The type of the current state in this option. */
    StateType subOptionStateType; /**< The type of the state of the last suboption executed (for action_done and action_aborted). */
    bool addedToGraph; /**< Was this option already added to the activation graph in this frame? */
    bool transitionExecuted; /**< Has a transition already been executed? True after a state change. */
    bool hasCommonTransition; /**< Does this option have a common transition? Is reset when entering the first state. */
  };

  /**
   * Instances of this class are passed as a default parameter to each option.
   * They maintain the current state of the option.
   */
  struct OptionExecution
  {
    const char* optionName; /**< The name of the option (for activation graph). */
    OptionContext& context; /**< The context of the state. */
    Cabsl* instance; /**< The object that encapsulates the behavior. */
    mutable std::vector<std::string> parameters; /**< Parameter names and their values. */

    /**
     * The constructor checks, whether the option was active in the previous frame and
     * if not, it switches back to the initial state.
     * @param optionName The name of the option (for activation graph).
     * @param context The context of the state.
     * @param instance The object that encapsulates the behavior.
     */
    OptionExecution(const char* optionName, OptionContext& context, Cabsl* instance) :
      optionName(optionName), context(context), instance(instance)
    {
      if(context.lastFrame != instance->lastFrameTime && context.lastFrame != instance->_currentFrameTime)
      {
        context.optionStart = instance->_currentFrameTime; // option started now
        context.stateStart = instance->_currentFrameTime; // initial state started now
        context.state = 0; // initial state is always marked with a 0
        context.stateType = OptionContext::normalState; // initial state is a normal state
        context.subOptionStateType = OptionContext::normalState; // reset action_done and action_aborted
      }
      context.addedToGraph = false; // not added to graph yet
      context.transitionExecuted = false; // no transition executed yet
      context.hasCommonTransition = false; // until one is found, it is assumed that there is no common transition
      if(instance->activationGraph)
        ++instance->activationGraph->currentDepth; // increase depth counter for activation graph
    }

    /**
     * The destructor cleans up the option context.
     */
    ~OptionExecution()
    {
      addToActivationGraph(); // add to activation graph if it has not been already
      if(instance->activationGraph)
        --instance->activationGraph->currentDepth; // decrease depth counter for activation graph
      context.subOptionStateType = instance->stateType; // remember the state type of the last sub option called
      instance->stateType = context.stateType; // publish the state type of this option, so the caller can grab it
      context.lastFrame = instance->_currentFrameTime; // Remember that this option was called in this frame
    }

    /**
     * The method is executed whenever the state is changed.
     * @param newState The new state to which it was changed.
     * @param stateType The type of the new state.
     */
    void updateState(int newState, typename OptionContext::StateType stateType) const
    {
      ASSERT(context.hasCommonTransition != context.transitionExecuted); // [common_]transition is missing
      context.transitionExecuted = true; // a transition was executed, do not execute another one
      if(context.state != newState) // ignore transitions that stay in the same state
      {
        context.state = newState;
        context.stateStart = instance->_currentFrameTime; // state started now
        context.stateType = stateType; // remember type of this state
      }
    }

    /**
     * Adds a string description containing the current value of a parameter to the list of parameters.
     * @param value The current value of the parameter.
     */
    void addParameter(const Streamable& value) const
    {
      OutMapMemory stream(true, 1024);
      stream << value;
      parameters.emplace_back(stream.data());
    }

    /**
     * The method adds information about the current option and state to the activation graph.
     * It suppresses adding it twice in the same frame.
     */
    void addToActivationGraph() const
    {
      if(!context.addedToGraph && instance->activationGraph)
      {
        instance->activationGraph->graph.emplace_back(optionName, instance->activationGraph->currentDepth,
                                                      context.stateName,
                                                      instance->_currentFrameTime - context.optionStart,
                                                      instance->_currentFrameTime - context.stateStart,
                                                      parameters);
        context.addedToGraph = true;
      }
    }
  };

  /** A class to store information about an option. */
  struct OptionDescriptor
  {
    const char* name; /**< The name of the option. */
    bool hasParameters; /**< Does the option have parameters?. */
    void (CabslBehavior::*option)(const OptionExecution&); /**< The option method. */
    size_t offsetOfContext; /**< The memory offset of the context within the behavior class. */
    int index; /**< The index of the option (for the enum of all options). */

    /** Default constructor, because STL types need one. */
    OptionDescriptor() = default;

    /**
     * Constructor.
     * @param name The name of the option.
     * @param hasParameters Does the option have parameters?
     * @param option The option method.
     * @param offsetOfContext The memory offset of the context within the behavior class.
     */
    OptionDescriptor(const char* name, bool hasParameters, void (CabslBehavior::*option)(const OptionExecution&), size_t offsetOfContext) :
      name(name), hasParameters(hasParameters), option(option), offsetOfContext(offsetOfContext), index(0)
    {}
  };

public:
  /** A class that collects information about all options in the behavior. */
  class OptionInfos
  {
  private:
    static std::vector<const OptionDescriptor*>* optionsByIndex; /**< All parameterless options in the sequence they were declared. */
    static std::unordered_map<std::string, const OptionDescriptor*>* optionsByName; /**< All parameterless options, indexed by their names. */

  public:
    enum Option : unsigned char {none}; /**< A dummy enum for all options. */

    /** The destructor frees the global object. */
    ~OptionInfos()
    {
      delete optionsByIndex;
      delete optionsByName;
      optionsByIndex = nullptr;
      optionsByName = nullptr;
    }

    /**
     * The method prepares the collection of information about all options in optionByIndex
     * and optionsByName. It also adds a dummy option descriptor at index 0 with the name "none".
     */
    static void init()
    {
      ASSERT(!optionsByIndex);
      ASSERT(!optionsByName);
      optionsByIndex = new std::vector<const OptionDescriptor*>;
      optionsByIndex->reserve(maxNumOfOptions);
      optionsByName = new std::unordered_map<std::string, const OptionDescriptor*>;
      static OptionDescriptor descriptor("none", false, 0, 0);
      optionsByIndex->push_back(&descriptor);
      (*optionsByName)[descriptor.name] = optionsByIndex->back();
      TypeRegistry::addEnum(typeid(Option).name());
      TypeRegistry::addEnumConstant(typeid(Option).name(), "none");
    }

    /**
     * The method adds information about an option to the collections.
     * It will be call from the constructors of static objects created for each
     * option.
     * Note that options with parameters will be ignored, because they currently
     * cannot be called externally.
     * @param descriptor A function that can return the description of an option.
     */
    static void add(OptionDescriptor& descriptor)
    {
      if(!optionsByIndex)
        init();

      ASSERT(optionsByIndex);
      ASSERT(optionsByName);
      if(!descriptor.hasParameters // ignore options with parameters for now
         && TypeRegistry::getEnumValue(typeid(Option).name(), descriptor.name) < 0) // only register once
      {
        descriptor.index = static_cast<int>(optionsByIndex->size());
        optionsByIndex->push_back(&descriptor);
        (*optionsByName)[descriptor.name] = optionsByIndex->back();
        TypeRegistry::addEnumConstant(typeid(Option).name(), descriptor.name);
      }
      ASSERT(optionsByIndex->size() <= maxNumOfOptions);
    }

    /**
     * The method returns the enum constant of an option.
     * @param name The name of the option.
     * @return The enum constant (i.e. the index) of an option. If the name is not known,
     *         the constant "none" will be returned.
     */
    static Option getOption(const char* name)
    {
      const auto i = optionsByName->find(name);
      if(i == optionsByName->end())
        return none;
      else
        return static_cast<Option>(i->second->index);
    }

    /**
     * The method executes a certain option. Note that only parameterless options can be
     * executed.
     * @param behavior The behavior instance.
     * @param option The index of the option.
     */
    static void execute(CabslBehavior* behavior, Option option)
    {
      if(!optionsByIndex)
        init();

      if(option != none && option < static_cast<int>(optionsByIndex->size()))
      {
        const OptionDescriptor& descriptor = *(*optionsByIndex)[option];
        OptionContext& context = *reinterpret_cast<OptionContext*>(reinterpret_cast<char*>(behavior) + descriptor.offsetOfContext);
        (behavior->*(descriptor.option))(OptionExecution(descriptor.name, context, behavior));
      }
    }
  };

protected:
  /**
   * A template class for collecting information about an option.
   * @tparam descriptor A function that can return the description of the option.
   */
  template<void*(*descriptor)()> class OptionInfo : public OptionContext
  {
  private:
    /** A helper structure to register information about the option. */
    struct Registrar
    {
      Registrar() {OptionInfos::add(reinterpret_cast<OptionDescriptor*(*)()>(descriptor));}
    };
    static Registrar registrar; /**< The instance that registers the information about the option through construction. */

  public:
    /** A dummy constructor that enforces linkage of the static member. */
    OptionInfo() {static_cast<void>(&registrar);}
  };

private:
  static OptionInfos collectOptions; /**< This global instantiation collects data about all options. */
  typename OptionContext::StateType stateType; /**< The state type of the last option called. */
  unsigned lastFrameTime; /**< The timestamp of the last time the behavior was executed. */
  ActivationGraph* activationGraph; /**< The activation graph for debug output. Can be zero if not set. */

protected:
  static thread_local Cabsl* _theInstance; /**< The instance of this behavior used. */
  unsigned _currentFrameTime; /**< The timestamp of the last time the behavior was executed. */

  /**
   * Constructor.
   * @param activationGraph When set, the activation graph will be filled with the
   *                        options and states executed in each frame.
   */
  Cabsl(ActivationGraph* activationGraph = nullptr) :
    stateType(OptionContext::normalState),
    lastFrameTime(0),
    activationGraph(activationGraph),
    _currentFrameTime(0)
  {
    static_cast<void>(&collectOptions); // Enforce linking of this global object
  }

public:
  /**
   * Must be call at the beginning of each behavior execution cycle.
   * @param frameTime The current time in ms.
   * @param clearActivationGraph Whether the activation graph should be cleared.
   */
  void beginFrame(unsigned frameTime, bool clearActivationGraph = true)
  {
    _currentFrameTime = frameTime;
    if(activationGraph && clearActivationGraph)
    {
      activationGraph->currentDepth = 0;
      activationGraph->graph.clear();
    }
    _theInstance = this;
  }

  /**
   * Execute an option as a root.
   * Several root options can be executed in a single behavior execution cycle.
   * @param root The root option that is executed.
   */
  void execute(typename OptionInfos::Option root)
  {
    OptionInfos::execute(static_cast<CabslBehavior*>(this), root);
  }

  /** Must be called at the end of each behavior execution cycle. */
  void endFrame()
  {
    _theInstance = nullptr;
    lastFrameTime = _currentFrameTime;
  }

  /** Enforces that all options will be reset to their initial states in their next execution. */
  void reset()
  {
    lastFrameTime = 0;
  }
};

template<typename CabslBehavior> thread_local Cabsl<CabslBehavior>* Cabsl<CabslBehavior>::_theInstance;
template<typename CabslBehavior> std::vector<const typename Cabsl<CabslBehavior>::OptionDescriptor*>* Cabsl<CabslBehavior>::OptionInfos::optionsByIndex;
template<typename CabslBehavior> std::unordered_map<std::string, const typename Cabsl<CabslBehavior>::OptionDescriptor*>* Cabsl<CabslBehavior>::OptionInfos::optionsByName;
template<typename CabslBehavior> typename Cabsl<CabslBehavior>::OptionInfos Cabsl<CabslBehavior>::collectOptions;

/**
 * The macro defines a state. It must be followed by a block of code that defines the state's body.
 * @param name The name of the state.
 */
#define state(name) _state(name, __LINE__, OptionContext::normalState)

/**
 * The macro defines a target state. It must be followed by a block of code that defines the state's body.
 * A parent option can check whether a target state has been reached through action_done.
 * @param name The name of the target state.
 */
#define target_state(name) _state(name, __LINE__, OptionContext::targetState)

/**
 * The macro defines an aborted state. It must be followed by a block of code that defines the state's body.
 * A parent option can check whether an aborted state has been reached through action_aborted.
 * @param name The name of the aborted state.
 */
#define aborted_state(name) _state(name, __LINE__, OptionContext::abortedState)

/**
 * The macro defines an option. It must be followed by a block of code that defines the option's body
 * The option gets an additional parameter that manages its context. If the option has parameters,
 * two methods are generated. The first one adds the parameters to the execution environment and calls
 * the second one.
 * @param ... The name of the option and an arbitrary number of parameters. They can include default
 *            parameters at the end. Their syntax is described at the beginning of this file.
 */
#define option(...) \
  _CABSL_OPTION_I(_CABSL_HAS_PARAMS(__VA_ARGS__), __VA_ARGS__)
#define _CABSL_OPTION_I(hasParams, ...) _CABSL_OPTION_II(hasParams, (__VA_ARGS__))
#define _CABSL_OPTION_II(hasParams, params) _CABSL_OPTION_##hasParams params

/** Macro specialization for option with parameters. */
#define _CABSL_OPTION_1(name, ...) _CABSL_OPTION_III(_STREAM_TUPLE_SIZE(__VA_ARGS__, ignore), name, __VA_ARGS__, ignore)

/** Generate an attribute declaration without an initialization. */
#define _CABSL_DECL_WITHOUT_INIT(seq) decltype(Streaming::TypeWrapper<_STREAM_DECL_I seq))>::type) _STREAM_VAR(seq);

/** Generate a parameter declaration with an initialization if avaliable. */
#define _CABSL_PARAM_WITH_INIT(seq) decltype(Streaming::TypeWrapper<_STREAM_DECL_I seq))>::type) _STREAM_VAR(seq) _STREAM_INIT(seq),

/** Generate a parameter declaration without an initialization. */
#define _CABSL_PARAM_WITHOUT_INIT(seq) decltype(Streaming::TypeWrapper<_STREAM_DECL_I seq))>::type) _STREAM_VAR(seq),

/** Generate a parameter declaration without an initialization (without comma). */
#define _CABSL_PARAM_WITHOUT_INIT2(seq) decltype(Streaming::TypeWrapper<_STREAM_DECL_I seq))>::type) _STREAM_VAR(seq)

/** Generate a variable name for the list of actual parameters of a method call. */
#define _CABSL_VAR(seq) _STREAM_VAR(seq),

/** Generate code for streaming a variable and adding it to the parameters stored in the execution environment. */
#define _CABSL_STREAM(seq) \
  _STREAM_JOIN(_CABSL_STREAM_, _STREAM_SEQ_SIZE(seq))(seq) \
  { \
    struct _S : public Streamable \
    { \
      _CABSL_DECL_WITHOUT_INIT(seq) \
      void serialize(In* in, Out* out) \
      { \
        auto _STREAM_VAR(seq) = this->_STREAM_VAR(seq); \
        _STREAM_SER(seq) \
      } \
      _S(_CABSL_PARAM_WITHOUT_INIT2(seq)) : _STREAM_VAR(seq)(_STREAM_VAR(seq)) {} \
    } _s(_STREAM_VAR(seq)); \
    _o.addParameter(_s); \
  }

#ifndef __INTELLISENSE__

/** Macro specialization for parameterless option. */
#define _CABSL_OPTION_0(name) \
  _CABSL_OPTION_V(name, false) \
  void name(const OptionExecution& _o = OptionExecution(#name, static_cast<CabslBehavior*>(_theInstance)->_##name##Context, _theInstance))
#define _CABSL_OPTION_III(n, name, ...) _CABSL_OPTION_IV(n, name, (_CABSL_PARAM_WITH_INIT, __VA_ARGS__), (_CABSL_VAR, __VA_ARGS__), (_CABSL_STREAM, __VA_ARGS__), (_CABSL_PARAM_WITHOUT_INIT, __VA_ARGS__))
#define _CABSL_OPTION_IV(n, name, params1, params2, params3, params4) \
  _CABSL_OPTION_V(name, true) \
  void name(_STREAM_ATTR_##n params1 const OptionExecution& _o = OptionExecution(#name, static_cast<CabslBehavior*>(_theInstance)->_##name##Context, _theInstance)) \
  { \
    _STREAM_ATTR_##n params3 \
    _##name(_STREAM_ATTR_##n params2 _o); \
  } \
  void _##name(_STREAM_ATTR_##n params4 const OptionExecution& _o)
#define _CABSL_OPTION_V(name, hasParams) \
  OptionContext _##name##Context; \
  static void _reg##name##Context() \
  { \
    PUBLISH(_reg##name##Context); \
    static OptionDescriptor descriptor(#name, hasParams, reinterpret_cast<void (CabslBehavior::*)(const OptionExecution&)>(&CabslBehavior::name), \
                                       reinterpret_cast<size_t>(&reinterpret_cast<CabslBehavior*>(16)->_##name##Context) - 16); \
    OptionInfos::add(descriptor); \
  }

/** If a default value exists, only stream parameters that are different from it. */
#define _CABSL_STREAM_1(seq)
#define _CABSL_STREAM_2(seq) if(!(_STREAM_VAR(seq) == _STREAM_INIT_I_2_I(seq)))

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
  _state(name, 0, OptionContext::normalState)

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
 * The macro marks a common_transition. It sets a flag so that a transition is accepted,
 * even if not executed through the keyword "transition".
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

#else // __INTELLISENSE__

#ifndef INTELLISENSE_PREFIX
#define INTELLISENSE_PREFIX
#endif
#define _CABSL_OPTION_0(name) void INTELLISENSE_PREFIX name()
#define _CABSL_OPTION_III(n, name, ...) _CABSL_OPTION_IV(n, name, (_CABSL_PARAM_WITH_INIT, __VA_ARGS__))
#define _CABSL_OPTION_IV(n, name, params) \
  void INTELLISENSE_PREFIX name(_STREAM_ATTR_##n params bool _ignore = false)

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

#endif

/** Check whether an option has parameters. */
#define _CABSL_HAS_PARAMS(...) _CABSL_HAS_PARAMS_I((__VA_ARGS__, _CABSL_HAS_PARAMS_II))
#define _CABSL_HAS_PARAMS_I(params) _STREAM_TUPLE_SIZE_II params
#define _CABSL_HAS_PARAMS_II \
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0
