/**
 * The file declares a class that instantiates all libraries.
 * @author Thomas Röfer
 */

#pragma once

#include "LibraryBase.h"
#include "Tools/RingBuffer.h"

namespace Behavior2015
{
#include "Libraries/LibTactic.h"
#include "Libraries/LibInformation.h"

  class Libraries : public BehaviorBase
  {
  private:
    static thread_local Libraries* theInstance;
    std::vector<LibraryBase*> libraries; /**< All the member libraries of this class. */

  public:
    LibInformation          LibInformation;           /**< Contains information about stuff */
    LibTactic               LibTactic;                /**< Contains methods that decide team tactic */

    Libraries(const BehaviorControl2015Base& base, BehaviorData& behaviorData);
    virtual ~Libraries() { theInstance = nullptr; }

    /**
     * Assignment operator, required by Cabsl.
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
