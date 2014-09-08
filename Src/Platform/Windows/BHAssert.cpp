/**
* @file Platform/Windows/BHAssert.cpp
* Some helper functions for low level debugging
* @author Colin Graf
*/

#ifndef NDEBUG

#include "BHAssert.h"

#include <cstdio>
#include <cstdarg>
#include <windows.h>

#ifdef WINDOWS
#define snprintf sprintf_s
#define vsnprintf vsprintf_s
#endif

void Assert::print(const char* file, int line, const char* format, ...)
{
  char data[320];
  int length;
  length = snprintf(data, sizeof(data) - 2, "%s:%d: ", file, line);
  if(length < 0)
    length = sizeof(data) - 2;
  va_list ap;
  va_start(ap, format);
  int i = vsnprintf(data + length, sizeof(data) - length - 2, format, ap);
  if(i < 0)
    length = sizeof(data) - 2;
  else
    length += i;
  va_end(ap);
  data[length++] = '\n';
  data[length] = '\0';
#ifdef WINDOWS
  OutputDebugString(data);
#else
  fputs(data, stderr);
  fflush(stderr);
#endif
}

#endif // NDEBUG
