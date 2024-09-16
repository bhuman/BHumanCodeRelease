#include "Output.h"

#if !(defined TARGET_ROBOT && defined NDEBUG)
#ifdef WINDOWS
#include <Windows.h>
#endif
#include <cstdio>

void Output::print(const char* message)
{
  fprintf(stderr, "%s\n", message);
#ifdef WINDOWS
  OutputDebugStringA(message);
  OutputDebugStringA("\n");
#endif
}
#endif
