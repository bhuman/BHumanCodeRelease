/**
 * The file implements the base class for all behavior libraries.
 * If a library is used by another library, it must be added here.
 * @author Thomas Röfer
 */

#include "Libraries.h"

namespace Behavior2015
{
  LibraryBase::LibraryBase() :
    BehaviorBase(*Libraries::theInstance),
    libCodeRelease(Libraries::theInstance->libCodeRelease)
  {
    Libraries::theInstance->libraries.push_back(this);
  }
}
