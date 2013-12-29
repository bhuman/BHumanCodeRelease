/**
 * @file Cabsl.h
 *
 * The file defines a set of macros that define the C-based abstract
 * behavior description language (CABSL). Semantically, it follows
 * the ideas of XABSL. It should be used inside a module that at
 * least requires FrameInfo.
 *
 * Grammar:
 *
 * <cabsl> ::= { <option> }
 *
 * <option> ::= option '(' <C-ident> { ',' <C-decl> } ')'
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
 * labels). Conditions can use the pre-defined
 * symbols 'state_time', 'option_time', 'action_done', and 'action_aborted'.
 * Within a state, the action <C-statements> can contain calls to
 * other (sub)options. 'action_done' determines whether the last suboption
 * called reached a target state in the previous execution cycle.
 * 'action_aborted' does the same for an aborted state.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Debugging/Debugging.h"
#include "Tools/Enum.h"

/**
 * The context stores the current state of an option.
 */
class OptionContext
{
public:
  /** The different types of states (for implementing target_state and aborted_state). */
  ENUM(StateType,
    normalState,
    targetState,
    abortedState
  );

  int state; /**< The state currently selected. This is actually the line number in which the state was declared. */
  const char* stateName; /**< The name of the state (for debug messages). */
  unsigned lastFrame; /**< The time stamp of the last frame in which this option was executed. */
  unsigned optionStart; /**< The time when the option started to run (for option_time). */
  unsigned stateStart; /**< The time when the current state started to run (for state_time). */
  StateType stateType; /**< The type of the current state in this option. */
  StateType subOptionStateType; /**< The type of the state of the last suboption executed (for action_done and action_aborted). */
  bool messageSent; /**< Was the debug message already sent in this frame? */
  bool transitionExecuted; /**< Has a transition already been executed? True after a state change. */
  bool hasCommonTransition; /**< Does this option have a common transition? Is reset when entering the first state. */

  OptionContext() : lastFrame(1) {}
};

/**
 * Instances of this class are passed as a default parameter to each option.
 * They maintain the current state of the option.
 */
class OptionExecution
{
public:
  const char* optionName; /**< The name of the option (for debug messages). */
  OptionContext& context; /**< The context of the state. */
  Behavior* instance; /**< The object that encapsulates the behavior. */

  /**
   * The constructor checks, whether the option was active in the previous frame and
   * if not, it switches back to the initial state.
   * @param optionName The name of the option (for debug messages).
   * @param context The context of the state.
   * @param instance The object that encapsulates the behavior.
   */
  OptionExecution(const char* optionName, OptionContext& context, Behavior* instance)
  : optionName(optionName), context(context), instance(instance)
  {
    if(context.lastFrame != instance->_lastFrameTime && context.lastFrame != instance->theFrameInfo.time)
    {
      context.optionStart = instance->theFrameInfo.time; // option started now
      context.stateStart = instance->theFrameInfo.time; // initial state started now
      context.state = 0; // initial state is always marked with a 0
      context.stateType = OptionContext::normalState; // initial state is a normal state
      context.subOptionStateType = OptionContext::normalState; // reset action_done and action_aborted
    }
    context.messageSent = false; // no debug message was sent yet
    context.transitionExecuted = false; // no transition executed yet
    context.hasCommonTransition = false; // until one is found, it is assumed that there is no common transition
    ++instance->_depth; // increase depth counter for debug messages
  }

  /**
   * The destructor cleans up the option context.
   */
  ~OptionExecution()
  {
    sendDebugMessage(); // send the debug message if it has not been already
    --instance->_depth; // decrease depth counter for debug messages
    context.subOptionStateType = instance->_stateType; // remember the state type of the last sub option called
    instance->_stateType = context.stateType; // publish the state type of this option, so the caller can grab it
    context.lastFrame = instance->theFrameInfo.time; // Remember that this option was called in this frame
  }

  /**
   * The method is executed whenever the state is changed.
   * @param newState The new state to which it was changed.
   * @param stateType The type of the new state.
   */
  void updateState(int newState, OptionContext::StateType stateType) const
  {
    ASSERT(context.hasCommonTransition != context.transitionExecuted); // [common_]transition is missing
    context.transitionExecuted = true; // a transition was executed, do not execute another one
    if(context.state != newState) // ignore transitions that stay in the same state
    {
      context.state = newState;
      context.stateStart = instance->theFrameInfo.time; // state started now
      context.stateType = stateType; // remember type of this state
    }
  }

  /**
   * The method sends a debug message about the current option and state if requested.
   * It suppresses sending the message twice in the same frame.
   */
  void sendDebugMessage() const
  {
    if(!context.messageSent)
    {
      instance->theActivationGraph.graph.push_back(ActivationGraph::Node(optionName, instance->_depth,
                                                                       context.stateName,
                                                                       instance->theFrameInfo.time - context.optionStart,
                                                                       instance->theFrameInfo.time - context.stateStart));
      context.messageSent = true;
    }
  }
};

/** A class to store information about an option. */
class OptionDescriptor
{
public:
  const char* name; /**< The name of the option. */
  const char* parameters; /**< All formal parameters as a single string. */
  void (Behavior::*option)(const OptionExecution&); /**< The option method. */
  size_t offsetOfContext; /**< The memory offset of the context within the behavior class. */
  int index; /**< The index of the option (for the enum of all options). */

  /**< Default constructor, because STL types need one. */
  OptionDescriptor() {}

  /**
   * Constructor.
   * @param name The name of the option.
   * @param parameters All formal parameters as a single string.
   * @param option The option method.
   * @param offsetOfContext The memory offset of the context within the behavior class.
   */
  OptionDescriptor(const char* name, const char* parameters, void (Behavior::*option)(const OptionExecution&), size_t offsetOfContext)
  : name(name), parameters(parameters), option(option), offsetOfContext(offsetOfContext), index(0) {}
};

/** A class that collects information about all options in the behavior. */
class OptionInfos
{
private:
  static std::vector<OptionDescriptor> optionsByIndex; /**< A parameterless options in the sequence they were declared. */
  static std::unordered_map<std::string, OptionDescriptor*> optionsByName; /**< All parameterless options, index by their names. */

public:
  enum Option : unsigned char {none}; /**< A dummy enum for all options. */

  /**
   * The constructor collects information about all options. It uses the assignment operators
   * of objects that were placed in the behavior to collect all data in optionByIndex and
   * optionsByName. It also adds a dummy option descriptor at index 0 with the name "none".
   */
  OptionInfos()
  {
    OptionDescriptor o("none", "", 0, 0);
    optionsByIndex.push_back(o);
    optionsByName[o.name] = &optionsByIndex.back();
    char buf[sizeof(Behavior)];
    // executes assignment operators -> recording information!
    (Behavior&) *buf = (const Behavior&) *buf;
  }

  /**
   * The method adds information about an option to the collections.
   * It will be call from the assignment operator of the objects
   * that were placed in the behavior.
   * Note that options with parameters will be ignore, because they currently
   * cannot be called externally.
   * @param descriptor A function that can return the description of an option.
   */
  static void add(OptionDescriptor (*descriptor)())
  {
    OptionDescriptor o = descriptor();
    if(!*o.parameters) // ignore options with parameters for now
    {
      o.index = optionsByIndex.size();
      optionsByIndex.push_back(o);
      optionsByName[o.name] = &optionsByIndex.back();
    }
  }

  /**
   * The method returns the name of an option.
   * It is required to make the option enum streamable.
   * @param option The index of the option.
   * @return The name of the option.
   */
  static const char* getName(Option option)
  {
    if((unsigned) option >= optionsByIndex.size())
      return 0;
    else
      return optionsByIndex[option].name;
  }

  /**
   * The method returns the enum constant of an option.
   * @param name The name of the option.
   * @return The enum constant (i.e. the index) of an option. If the name is not known,
   *         the constant "none" will be returned.
   */
  static Option getOption(const char* name)
  {
    std::unordered_map<std::string, OptionDescriptor*>::const_iterator i = optionsByName.find(name);
    if(i == optionsByName.end())
      return none;
    else
      return (Option) i->second->index;
  }

  /**
   * The method executes a certain option. Note that only parameterless options can be
   * executed.
   * @param behavior The behavior instance.
   * @param option The index of the option.
   */
  static void execute(Behavior* behavior, Option option)
  {
    if(option != none && option < (int) optionsByIndex.size())
    {
      const OptionDescriptor& descriptor = optionsByIndex[option];
      OptionContext& context = *(OptionContext*) ((char*) behavior + descriptor.offsetOfContext);
      (behavior->*(descriptor.option))(OptionExecution(descriptor.name, context, behavior));
    }
  }
};

/**
 * A template class for collecting information about an option.
 * @param descriptor A function that can return the description of the option.
 */
template<OptionDescriptor (*descriptor)()> class OptionInfo : public OptionContext
{
public:
  /**
   * The assignment operator publishes the address of the function that
   * can provide a description of the option.
   */
  void operator=(const OptionInfo&) {OptionInfos::add(descriptor);}
};

// Helpers to suppress the comma in a parameter list when __VA_ARGS__ is empty
// Note: if an option has more than 10 parameters, these macros have to be extended
#ifdef WIN32
#undef _CABSL_COMMA
#define _CABSL_COMMA(x) _CABSL_COMMA2(x)
#define _CABSL_COMMA2(x) _CABSL_GETCOMMA5(_CABSL_COMMA_, x)
#define _CABSL_COMMA_0
#define _CABSL_COMMA_1 ,
#define _CABSL_GETCOMMA(...) _CABSL_GETCOMMA5(_CABSL_COMMA, (_CABSL_GETCOMMA3(_CABSL_GETCOMMA2, (0, __VA_ARGS__))))
#define _CABSL_GETCOMMA2(...) __VA_ARGS__
#define _CABSL_GETCOMMA3(a, b) _CABSL_GETCOMMA4(_CABSL_GETCOMMA6, (a b, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0))
#define _CABSL_GETCOMMA4(a, b) a b
#define _CABSL_GETCOMMA5(a, b) a##b
#define _CABSL_GETCOMMA6(x, a, b, c, d, e, f, g, h, i, j, ...) j
#else
#define _CABSL_COMMA ,
#define _CABSL_GETCOMMA2(x, a, b, c, d, e, f, g, h, i, j, ...) j
#define _CABSL_GETCOMMA(...) _CABSL_GETCOMMA2(0,##__VA_ARGS__, _CABSL_COMMA, _CABSL_COMMA, _CABSL_COMMA, _CABSL_COMMA, _CABSL_COMMA, _CABSL_COMMA, _CABSL_COMMA, _CABSL_COMMA, _CABSL_COMMA, )
#endif

/**
 * The macro defines an option. It must be followed by a block of code that defines the option's body
 * The option gets an additional parameter that manages its context.
 * @param name The name of the option.
 * @param ... An arbitrary number of parameters. They can include default parameters at the end.
 */
#define option(name, ...) \
  static OptionDescriptor _get##name##Descriptor() {return OptionDescriptor(#name, "" #__VA_ARGS__, (void (Behavior::*)(const OptionExecution&)) &Behavior::name, ((size_t) &((Behavior*) 16)->_##name##Context) - 16);} \
  OptionInfo<&Behavior::_get##name##Descriptor> _##name##Context; \
  void name(__VA_ARGS__ _CABSL_GETCOMMA(__VA_ARGS__) \
            const OptionExecution& _o = OptionExecution(#name, _theInstance->_##name##Context, _theInstance))

/**
 * The macro defines a state. It must be followed by a block of code that defines the state's body.
 * @param name The name of the state.
 */
#define state(name) _state(name, __LINE__, OptionContext::normalState)

/**
 * The macro defines an initial state. It must be followed by a block of code that defines the state's body.
 * Since there does not need to be a transition to an initial state, an unreachable goto statement is defined
 * to avoid warnings about unused labels. The initial state also has an unused extra label that simply ensures
 * that whenever there are states, there must be exactly one initital state.
 * @param name The name of the initial state.
 */
#define initial_state(name) \
  initial_state: \
  if(_o.context.state == -1) \
    goto name; \
  _state(name, 0, OptionContext::normalState)

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
  if(_o.context.state == line && (_o.context.stateName = #name))

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
  if(!_o.context.transitionExecuted++)

/**
 * The macro marks an action. It should be followed by a block of code that contains the
 * implementation of the action.
 * The macro sends the debug message about the current option and state, so that it is
 * sent before any suboptions called in the action block can send theirs.
 */
#define action _o.sendDebugMessage();

/** The time since the execution of this option started. */
#define option_time int(theFrameInfo.time - _o.context.optionStart)

/** The time since the execution of the current state started. */
#define state_time int(theFrameInfo.time - _o.context.stateStart)

/** Did a suboption called reached a target state? */
#define action_done (_o.context.subOptionStateType == OptionContext::targetState)

/** Did a suboption called reached an aborted state? */
#define action_aborted (_o.context.subOptionStateType == OptionContext::abortedState)
