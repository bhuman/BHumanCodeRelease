/**
* @file Platform/Win32/BHAssert.cpp
* Some helper functions for low level debugging
* @author Colin Graf
*/

#ifndef NDEBUG

#include "BHAssert.h"

#include <cstdio>
#include <cstdarg>
#include <windows.h>

void Assert::print(const char* file, int line, const char* format, ...)
{
  char data[320];
  int length;
#ifdef _MSC_VER
  length = sprintf_s(data, sizeof(data) - 1, "%s:%d: ", file, line);
#else
  length = ::snprintf(data, sizeof(data) - 2, "%s:%d: ", file, line);
  if(length < 0)
    length = sizeof(data) - 2;
#endif
  va_list ap;
  va_start(ap, format);
#ifdef _MSC_VER
  length += vsprintf_s(data + length, sizeof(data) - length - 1, format, ap);
#else
  int i = ::vsnprintf(data + length, sizeof(data) - length - 2, format, ap);
  if(i < 0)
    length = sizeof(data) - 2;
  else
    length += i;
#endif
  va_end(ap);
  data[length++] = '\n';
  data[length] = '\0';
#ifdef _WIN32
  OutputDebugString(data);
#else
  fputs(data, stderr);
  fflush(stderr);
#endif
}

#endif // NDEBUG
