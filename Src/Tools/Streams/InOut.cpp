/**
 * @file Tools/Streams/InOut.cpp
 *
 * Implementation of the streamable function endl.
 *
 * @author <a href="mailto:oberlies@sim.tu-darmstadt.de">Tobias Oberlies</a>
 */

#include "InOut.h"
#include <cstring>

Out& endl(Out& out)
{
  out.outEndL();
  return out;
}

In& endl(In& in)
{
  in.inEndL();
  return in;
}

namespace Streaming
{
  void trimName(const char*& name)
  {
    if(name)
    {
      const char* p = name + strlen(name) - 1;
      while(p >= name && *p != ')' && *p != ' ')
        --p;
      name = p + 1;
    }
  }
}
