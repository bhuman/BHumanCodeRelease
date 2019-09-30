/**
 * @file CabslCard.h
 *
 * This file defines macros to allow writing CABSL state machines in cards.
 * CABSL in cards does not know target states and aborted states.
 * This file cannot be included when Cabsl.h is already included.
 *
 * @author Arne Hasselbring
 * @author Thomas Röfer
 */

#include "Platform/BHAssert.h"
#include "Tools/BehaviorControl/Framework/BehaviorContext.h"

namespace CabslCard
{
  struct OptionContext
  {
    int state; /**< The state currently selected. This is actually the line number in which the state was declared. */
    bool transitionExecuted; /**< Has a transition already been executed? True after a state change. */
    bool hasCommonTransition; /**< Does this option have a common transition? Is reset when entering the first state. */

    void updateState(int newState, unsigned currentFrameTime, unsigned& stateStart)
    {
      ASSERT(hasCommonTransition != transitionExecuted); // [common_]transition is missing
      transitionExecuted = true; // a transition was executed, do not execute another one
      if(state != newState) // ignore transitions that stay in the same state
      {
        state = newState;
        stateStart = currentFrameTime; // state started now
      }
    }
  };
}

#define option \
  CabslCard::OptionContext _theCabslContext; \
  void reset() override \
  { \
    _theCabslContext.state = 0; \
    _context.stateStart = _CARD_REGISTRY::theInstance->currentFrameTime; \
  } \
  void execute() override \
  { \
    _theCabslContext.transitionExecuted = false; \
    _theCabslContext.hasCommonTransition = false; \
    cabslOption(_theCabslContext); \
  } \
  void cabslOption(CabslCard::OptionContext& _o)


/**
 * The macro defines a state. It must be followed by a block of code that defines the state's body.
 * @param name The name of the state.
 */
#define state(name) _state(name, __LINE__)

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
  _state(name, 0)

/**
 * The actual code generated for the state macros above.
 * An unreachable goto statement ensures that there is an initial state.
 * @param name The name of the state.
 * @param line The line number in which the state is defined of 0 for the initial state.
 */
#define _state(name, line) \
  if(false) \
  { \
    goto initial_state; \
  name: _o.updateState(line, _CARD_REGISTRY::theInstance->currentFrameTime, _context.stateStart); \
  } \
  _o.hasCommonTransition = false; \
  if(_o.state == line && (_context.stateName = #name) && (BH_TRACE, true))

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
#define option_time static_cast<int>(_CARD_REGISTRY::theInstance->currentFrameTime - _context.behaviorStart)

/** The time since the execution of the current state started. */
#define state_time static_cast<int>(_CARD_REGISTRY::theInstance->currentFrameTime - _context.stateStart)
