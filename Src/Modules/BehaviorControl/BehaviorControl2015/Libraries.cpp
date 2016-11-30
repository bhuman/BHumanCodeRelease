/**
 * The file implements a class that instantiates all libraries.
 * @author Thomas Röfer
 */

#include <cstring>
#include "Libraries.h"

namespace Behavior2015
{
  thread_local Libraries* Libraries::theInstance = nullptr;

  Libraries::Libraries(const BehaviorControl2015Base& base, BehaviorData& behaviorData) :
    BehaviorBase((theInstance = this, base), behaviorData)
  {}

  void Libraries::operator=(const Libraries& other)
  {
    if(this != &other)
      memcpy((void*) this, (void*) &other, sizeof(*this));
  }

  void Libraries::preProcessLibraries()
  {
    for(LibraryBase* library : libraries)
      library->preProcess();
  }

  void Libraries::postProcessLibraries()
  {
    for(LibraryBase* library : libraries)
      library->postProcess();
  }
}
