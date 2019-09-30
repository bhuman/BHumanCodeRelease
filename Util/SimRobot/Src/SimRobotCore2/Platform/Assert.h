/**
* @file Assert.h
* Declaration of class Assert
* @author Colin Graf
*/

#pragma once

#ifdef ASSERT
#error ASSERT already defined
#endif

#ifndef NDEBUG

#include <cassert>

/**
* @class Assert
* Class with helper function for low level debugging
*/
class Assert
{
public:
  /**
  * Prints a message to a debug output stream (e.g. stderr or OutputDebugString)
  * @param format The format of the message (printf-style)
  * @param ... Arguments for \c format
  */
  static void trace(const char* format, ...);
};

#define ASSERT(e) assert(e)
#define VERIFY(e) ASSERT(e)
#define TRACE(...) Assert::trace(__VA_ARGS__)

#else

#define ASSERT(e) static_cast<void>(0)
#define VERIFY(e) static_cast<void>(e)
#define TRACE(...) static_cast<void>(0)

#endif
