/**
 * The file declares the base class for all behavior libraries.
 * If a library is used by another library, it must be added here.
 * @author Thomas Röfer
 */

#pragma once

#include "BehaviorControl2015.h"

namespace Behavior2015
{
  class LibraryBase : public BehaviorBase
  {
  public:

    /**
     * The default constructor initializes all references with the actual libraries.
     * Note that these libraries may not be initialzed yet, so do not call them
     * during construction.
     */
    LibraryBase();

    /**
     * The method is called each time before a behavior cycle is executed.
     * It is supposed to be overridden.
     */
    virtual void preProcess() {}

    /**
     * The method is called each time after a behavior cycle was executed.
     * It is supposed to be overridden.
     */
    virtual void postProcess() {}
  };
}
