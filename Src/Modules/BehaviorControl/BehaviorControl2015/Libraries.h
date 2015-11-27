/**
 * The file declares a class that instantiates all libraries.
 * @author Thomas Röfer
 */

#pragma once

#include "LibraryBase.h"

namespace Behavior2015
{
#include "Libraries/LibCodeRelease.h"

  class Libraries : public BehaviorBase
  {
  private:
    static PROCESS_LOCAL Libraries* theInstance;
    std::vector<LibraryBase*> libraries; /**< All the member libraries of this class. */

  public:
    LibCodeRelease libCodeRelease; /**< Contains miscellaneous helper methods */

    Libraries(const BehaviorControl2015Base& base,
              BehaviorData& behaviorData);

    /**
     * Assignment operator, because the standard operator is not accepted by the compiler.
     * @param other The instance that is cloned.
     */
    void operator=(const Libraries& other);

    /** Calls the preProcess() method of each member library */
    void preProcessLibraries();

    /** Calls the postProcess() method of each member library */
    void postProcessLibraries();

    friend class LibraryBase;
  };
}
