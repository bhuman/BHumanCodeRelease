/**
 * @file Platform/Linux/BHAssert.cpp
 * Some helper functions for low level debugging
 * @author Colin Graf
 */

#include "Platform/BHAssert.h"

#ifndef NDEBUG

#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include <iostream>

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
  fputs(data, stderr);
  fflush(stderr);
}

void Assert::print(const char* file, int line, const std::string& message)
{
  std::cerr << file << ":" << line << ": " << message << std::endl;
}

void Assert::abort()
{
  ::abort();
}

#endif // NDEBUG
