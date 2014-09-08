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

#ifdef OSX
// Prevent strange recursive include: <assert.h> is resolved by this file!
#include <../include/assert.h>
#else
#include <cassert>
#endif
#define ASSERT(e) assert(e)
#define VERIFY(e) ASSERT(e)
#define TRACE(...) Assert::trace(__VA_ARGS__)

#else
#define ASSERT(e) ((void)0)
#define VERIFY(e) ((void)(e))
#define TRACE(...) ((void)0)
#endif
