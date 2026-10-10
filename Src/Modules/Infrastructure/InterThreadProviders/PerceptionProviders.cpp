/**
 * @file PerceptionProviders.cpp
 *
 * This file implements all modules that provide representations from perception
 * for the current Cognition frame.
 *
 * @author Thomas Röfer
 */

#include "PerceptionProviders.h"
#include "Framework/Module.h"

// The perception threads already draw these representations
#undef _MODULE_DRAW
#define _MODULE_DRAW(...)

// Define a module that provides a representation from a specific thread.
// This definition allows to insert an extra requirement and additional code.
#define ALIAS_FROM_THREAD_EXT(Representation, Thread, requirement, code) \
  MODULE(Thread##Representation##Provider, \
  {, \
    REQUIRES(Thread##Representation), \
    requirement \
    PROVIDES(Representation), \
  }); \
  \
  class Thread##Representation##Provider : public Thread##Representation##ProviderBase \
  { \
    void update(Representation& the##Representation) override \
    { \
      the##Representation = the##Thread##Representation; \
      code \
    } \
  }; \
  \
  MAKE_MODULE(Thread##Representation##Provider)

// Define a module that provides a representation from a specific thread.
#define ALIAS_FROM_THREAD(Representation, Thread) ALIAS_FROM_THREAD_EXT(Representation, Thread,,)

ALIAS_FROM_THREAD(FrameInfo, Color);
