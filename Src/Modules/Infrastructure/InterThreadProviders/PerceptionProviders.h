/**
 * @file PerceptionProviders.h
 *
 * This file declares all modules that provide representations from perception
 * for the current Cognition frame.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"

// Declare aliases of representation for specific threads
#define DECLARE(Representation, Thread) \
  STREAMABLE_WITH_BASE(Thread##Representation, Representation, {, }); \

// Needed by Cognition.cpp
DECLARE(FrameInfo, Color);
