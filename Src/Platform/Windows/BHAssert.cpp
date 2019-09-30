/**
 * @file Platform/Windows/BHAssert.cpp
 * Some helper functions for low level debugging
 * @author Colin Graf
 */

#ifndef NDEBUG

#include "Platform/BHAssert.h"

#include <cstdio>
#include <Windows.h>

void Assert::print(const char* file, int line, const char* format, ...)
{
  char data[320];
  int length = std::snprintf(data, sizeof(data) - 2, "%s:%d: ", file, line);
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
#ifdef TARGET_TOOL
  fputs(data, stderr);
  fflush(stderr);
#else
  OutputDebugString(data);
#endif
}

void Assert::print(const char* file, int line, const std::string& message)
{
  const std::string expandedMessage = std::string(file) + ":" + std::to_string(line) + ": " + message + "\n";
  OutputDebugString(expandedMessage.c_str());
}

void Assert::abort()
{
  __debugbreak();
};

#endif // NDEBUG
