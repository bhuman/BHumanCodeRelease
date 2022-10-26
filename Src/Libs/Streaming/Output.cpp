#include "Output.h"

#if !(defined TARGET_ROBOT && defined NDEBUG)
#include <cstdio>

void Output::print(const char* message)
{
  fprintf(stderr, "%s\n", message);
}
#endif
