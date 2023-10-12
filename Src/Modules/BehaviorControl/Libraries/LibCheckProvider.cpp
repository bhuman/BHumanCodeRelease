/**
 * @file LibCheckProvider.cpp
 *
 * Performs some checks of the behavior control module such as
 * - How often is a MotionRequest set?
 * - How often is a HeadMotionRequest set?
 *
 * @author Tim Laue
 */

#include "LibCheckProvider.h"

MAKE_MODULE(LibCheckProvider);

void LibCheckProvider::update(LibCheck& libCheck)
{
  reset();
  libCheck.inc = [this](LibCheck::CheckedOutput outputToCheck) {inc(outputToCheck);};
  libCheck.dec = [this](LibCheck::CheckedOutput outputToCheck) {dec(outputToCheck);};
  libCheck.performCheck = [this] {performCheck();};
}

void LibCheckProvider::reset()
{
  FOREACH_ENUM(LibCheck::CheckedOutput, i)
    callCounters[i] = 0;
}

void LibCheckProvider::performCheck() const
{
  const std::string options = getActivationGraphString(theActivationGraph);

  // Output counting checks:
  FOREACH_ENUM(LibCheck::CheckedOutput, i)
  {
    // Check, if output has been set at least once:
    if(callCounters[i] == 0 && notSetReaction[i] == warn)
    {
      OUTPUT_TEXT("Meeek! " << TypeRegistry::getEnumName(i) << " has not been set in this cycle (Robot " << theGameState.playerNumber
                  << (options == "" ? "" : ", Options: " + options) << ") !");
    }
    else if(notSetReaction[i] == abort)
    {
      ASSERT(callCounters[i] > 0);
    }
  }
}

void LibCheckProvider::inc(LibCheck::CheckedOutput outputToCheck)
{
  ++callCounters[outputToCheck];

  // Check, if output has not been set more than once:
  if(callCounters[outputToCheck] > 1)
  {
    if(multipleSetReaction[outputToCheck] == warn)
    {
      const std::string options = getActivationGraphString(theActivationGraph);

      OUTPUT_TEXT("Meeek! " << TypeRegistry::getEnumName(static_cast<LibCheck::CheckedOutput>(outputToCheck)) << " has been set more than once in this cycle (Robot "
                  << theGameState.playerNumber
                  << (options == "" ? "" : ", Options: " + options) << ") !");
    }
    else if(multipleSetReaction[outputToCheck] == abort)
    {
      ASSERT(callCounters[outputToCheck] <= 1);
    }
  }
}

void LibCheckProvider::dec(LibCheck::CheckedOutput outputToCheck)
{
  --callCounters[outputToCheck];
}

std::string LibCheckProvider::getActivationGraphString(const ActivationGraph& activationGraph) const
{
  std::string options = "";
  for(const auto& node : activationGraph.graph)
    options += (options == "" ? "" : ", ") + node.option + (node.state == "" ? "" : "/" + node.state);
  return options;
}
