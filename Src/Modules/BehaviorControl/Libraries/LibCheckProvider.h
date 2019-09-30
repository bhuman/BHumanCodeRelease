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

#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Module/Module.h"

MODULE(LibCheckProvider,
{,
  USES(ActivationGraph),
  REQUIRES(FrameInfo),
  REQUIRES(RobotInfo),
  USES(TeamActivationGraph),
  USES(TeamBehaviorStatus),
  PROVIDES(LibCheck),
  LOADS_PARAMETERS(
  {,
    (std::vector<int>) notSetCheck,       /** Assert that a request has been set at least once */
    (std::vector<int>) multipleSetCheck,  /** Print a warning if an assert has not been set at least once */
    (bool) assertValidWalkRequest,        /** Asserts that there are no strange walk parameters */
  }),
});

class LibCheckProvider : public LibCheckProviderBase
{
private:
  int callCounters[LibCheck::numOfCheckedOutputs]; /**< The counters for different checks */
  bool setArmsInThisFrame[Arms::numOfArms]; /**< This arm was set in this frame */

  /**
   * Updates LibCheck
   * @param libCheck The representation provided
   */
  void update(LibCheck& libCheck) override;

  /** Resets all status information */
  void reset();

  /**
   * Checks whether a behavior part set all its outputs
   * @param activationGraph The activation graph of the behavior part
   * @param start The first output ID to be checked
   * @param end The first output ID not to be checked after \c start
   */
  void checkOutputs(const ActivationGraph& activationGraph, LibCheck::CheckedOutput start, LibCheck::CheckedOutput end) const;

  /**
   * Checks whether the motion request is valid
   * @param activationGraph The activation graph of the individual behavior
   * @param theMotionRequest The motion request to check for validity
   */
  void checkMotionRequest(const ActivationGraph& activationGraph, const MotionRequest& theMotionRequest) const;

  /** Increments one counter */
  void inc(LibCheck::CheckedOutput outputToCheck);

  /**
   * Serializes an activation graph to a string
   * @param activationGraph The activation graph to convert
   * @return The compressed string that represents the activation graph
   */
  std::string getActivationGraphString(const ActivationGraph& activationGraph) const;
};
