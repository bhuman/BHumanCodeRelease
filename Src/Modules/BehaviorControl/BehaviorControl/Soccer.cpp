/**
 * @file Soccer.cpp
 *
 * This file implements the root behavior option (playing soccer).
 *
 * @author Arne Hasselbring
 */

#include "Soccer.h"

#include "Tools/BehaviorOptionRegistry.h"

MAKE_BEHAVIOR_OPTION(Soccer);

Soccer::Soccer() :
  Cabsl<Soccer>(BehaviorOptionRegistry::theInstance->theActivationGraph)
{
}

void Soccer::execute()
{
  beginFrame(theFrameInfo.time, false);
  Cabsl<Soccer>::execute(OptionInfos::getOption("Root"));
  endFrame();
}
