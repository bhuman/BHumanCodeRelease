/**
 * The file implements a class that instantiates all libraries.
 * @author Thomas Röfer
 */

#include <cstring>
#include "Libraries.h"

namespace Behavior2013
{
  PROCESS_WIDE_STORAGE(Libraries) Libraries::theInstance = 0;

  Libraries::Libraries(const BehaviorControl2013Base& base,
                       BehaviorControlOutput& behaviorControlOutput) :
    BehaviorBase((theInstance = this, base), behaviorControlOutput)
  {}

  void Libraries::operator=(const Libraries& other)
  {
    memcpy(this, &other, sizeof(*this));
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
