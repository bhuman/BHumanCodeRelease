/**
 * @file LibCheckProvider.h
 *
 * Performs some checks of the behavior control module such as
 * - How often is a MotionRequest set?
 * - How often is a HeadMotionRequest set?
 *
 * @author Tim Laue
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Infrastructure/GameState.h"
#include "Streaming/EnumIndexedArray.h"

MODULE(LibCheckProvider,
{,
  USES(ActivationGraph),
  REQUIRES(GameState),
  PROVIDES(LibCheck),
  LOADS_PARAMETERS(
  {
    ENUM(Reaction,
    {,
      ignore,
      warn,
      abort,
    }),

    (ENUM_INDEXED_ARRAY(Reaction, LibCheck::CheckedOutput)) notSetReaction,       /**< Assert that a request has been set at least once */
    (ENUM_INDEXED_ARRAY(Reaction, LibCheck::CheckedOutput)) multipleSetReaction,  /**< Print a warning if an assert has not been set at least once */
  }),
});

class LibCheckProvider : public LibCheckProviderBase
{
private:
  ENUM_INDEXED_ARRAY(int, LibCheck::CheckedOutput) callCounters; /**< The counters for different checks */

  /**
   * Updates LibCheck
   * @param libCheck The representation provided
   */
  void update(LibCheck& libCheck) override;

  /** Resets all status information */
  void reset();

  /** Checks whether a behavior part set all its outputs */
  void performCheck() const;

  /** Increments one counter */
  void inc(LibCheck::CheckedOutput outputToCheck);

  /** Decrements one counter. Only use if an output should really be overwritten. */
  void dec(LibCheck::CheckedOutput outputToCheck);

  /**
   * Serializes an activation graph to a string
   * @param activationGraph The activation graph to convert
   * @return The compressed string that represents the activation graph
   */
  std::string getActivationGraphString(const ActivationGraph& activationGraph) const;
};
