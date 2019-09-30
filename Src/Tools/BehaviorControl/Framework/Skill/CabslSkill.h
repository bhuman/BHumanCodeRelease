/**
 * @file CabslSkill.h
 *
 * This file defines macros to allow writing CABSL state machines in skill implementations.
 * The largest difference to regular CABSL is that action_done and action_aborted are missing.
 * isDone and isAborted of the called skills should be used directly instead (although there is
 * a slight difference in the semantics if skills are called more than once per frame).
 * This file cannot be included when Cabsl.h is already included.
 *
 * @author Arne Hasselbring
 * @author Thomas Röfer
 */

#include "Platform/BHAssert.h"
#include "Tools/BehaviorControl/Framework/BehaviorContext.h"

namespace CabslSkill
{
  struct OptionContext
  {
    /** The different types of states (for implementing target_state and aborted_state). */
    enum StateType
    {
      normalState,
      targetState,
      abortedState
    };

    int state; /**< The state currently selected. This is actually the line number in which the state was declared. */
    StateType stateType; /**< The type of the current state in this option. */
    bool transitionExecuted; /**< Has a transition already been executed? True after a state change. */
    bool hasCommonTransition; /**< Does this option have a common transition? Is reset when entering the first state. */

    void updateState(int newState, StateType stateType, unsigned currentFrameTime, unsigned& stateStart)
    {
      ASSERT(hasCommonTransition != transitionExecuted); // [common_]transition is missing
      transitionExecuted = true; // a transition was executed, do not execute another one
      if(state != newState) // ignore transitions that stay in the same state
      {
        state = newState;
        stateStart = currentFrameTime; // state started now
        this->stateType = stateType; // remember type of this state
      }
    }
  };
}

#define option(name) \
  CabslSkill::OptionContext _the##name##Context; \
  bool isDone(const name& p) const override { return (p._context.lastFrame == _SKILL_REGISTRY::theInstance->lastFrameTime || p._context.lastFrame == _SKILL_REGISTRY::theInstance->currentFrameTime) && _the##name##Context.stateType == CabslSkill::OptionContext::targetState; } \
  bool isAborted(const name& p) const override { return (p._context.lastFrame == _SKILL_REGISTRY::theInstance->lastFrameTime || p._context.lastFrame == _SKILL_REGISTRY::theInstance->currentFrameTime) && _the##name##Context.stateType == CabslSkill::OptionContext::abortedState; } \
  void reset(const name& p) override \
  { \
    _the##name##Context.state = 0; \
    _the##name##Context.stateType = CabslSkill::OptionContext::normalState; \
    p._context.stateStart = _SKILL_REGISTRY::theInstance->currentFrameTime; \
  } \
  void execute(const name& p) override \
  { \
    _the##name##Context.transitionExecuted = false; \
    _the##name##Context.hasCommonTransition = false; \
    cabslOptionFor##name(p, _the##name##Context); \
  } \
  void cabslOptionFor##name(const name& p, CabslSkill::OptionContext& _o)


/**
 * The macro defines a state. It must be followed by a block of code that defines the state's body.
 * @param name The name of the state.
 */
#define state(name) _state(name, __LINE__, CabslSkill::OptionContext::normalState)

/**
 * The macro defines a target state. It must be followed by a block of code that defines the state's body.
 * A calling skill can check whether a target state has been reached through isDone.
 * @param name The name of the target state.
 */
#define target_state(name) _state(name, __LINE__, CabslSkill::OptionContext::targetState)

/**
 * The macro defines an aborted state. It must be followed by a block of code that defines the state's body.
 * A calling skill can check whether an aborted state has been reached through isAborted.
 * @param name The name of the aborted state.
 */
#define aborted_state(name) _state(name, __LINE__, CabslSkill::OptionContext::abortedState)

/**
 * The macro defines an initial state. It must be followed by a block of code that defines the state's body.
 * Since there does not need to be a transition to an initial state, an unreachable goto statement is defined
 * to avoid warnings about unused labels. The initial state also has an unused extra label that simply ensures
 * that whenever there are states, there must be exactly one initial state.
 * @param name The name of the initial state.
 */
#define initial_state(name) \
  initial_state: \
  if(_o.state == -1) \
    goto name; \
  _state(name, 0, CabslSkill::OptionContext::normalState)

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
  name: _o.updateState(line, stateType, _SKILL_REGISTRY::theInstance->currentFrameTime, p._context.stateStart); \
  } \
  _o.hasCommonTransition = false; \
  if(_o.state == line && (p._context.stateName = #name) && (BH_TRACE, true))

/**
 * The macro marks a common_transition. It sets a flag so that a transition is accepted,
 * even if not executed through the keyword "transition".
 */
#define common_transition \
  _o.hasCommonTransition = true;

/**
 * The macro marks a transition. It should be followed by a block of code that contains the decision
 * tree that implements the transition.
 * The macro prevents the option from executing more than one transition per frame.
 * Setting the flag here also allows the transition target to check whether actually
 * a transition block was used to jump to the new state.
 */
#define transition \
  if((_o.transitionExecuted ^= true))

/**
 * The macro marks an action. It should be followed by a block of code that contains the
 * implementation of the action.
 */
#define action

/** The time since the execution of this option started. */
#define option_time static_cast<int>(_SKILL_REGISTRY::theInstance->currentFrameTime - p._context.behaviorStart)

/** The time since the execution of the current state started. */
#define state_time static_cast<int>(_SKILL_REGISTRY::theInstance->currentFrameTime - p._context.stateStart)
