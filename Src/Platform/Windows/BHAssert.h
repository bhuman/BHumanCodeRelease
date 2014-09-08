/**
* @file Platform/Windows/BHAssert.h
* This file contains macros for low level debugging
* @author Thomas Röfer
* @author Colin Graf
*/

#pragma once

#undef ASSERT
#undef VERIFY
#undef TRACE

/**
* Some tools for low level debugging
*/
class Assert
{
public:
#ifndef NDEBUG
  /**
  * Prints a formated message to stdout
  * @param file The name of the current file (__FILE__)
  * @param line The current file line (__LINE__)
  * @param format The format of the message to be printed
  */
  static void print(const char* file, int line, const char* format, ...);

  /**
  * Aborts the execution of the program
  */
  static void abort()
  {
#ifdef _MSC_VER
    __debugbreak();
#else
    __asm {int 3};
#endif
  };
#endif
};

/**
* ASSERT prints a message if \c cond is \c false and NDEBUG is not defined.
* ASSERT does not evaluate \c cond if NDEBUG is defined.
* @param cond The condition to be checked.
*/
#ifdef NDEBUG
#define ASSERT(cond) ((void)0)
#else
#define ASSERT(cond) ((void)((cond) ? 0 : (Assert::abort(), 0)))
#endif

/**
* VERIFY prints a message if \cond cond is \c false and NDEBUG is not defined.
* VERIFY does evaluate \c cond even if NDEBUG is defined.
* @param c The condition to be checked.
*/
#ifdef NDEBUG
#define VERIFY(cond) ((void)(cond))
#else
#define VERIFY(cond) ((void)((cond) ? 0 : (Assert::abort(), 0)))
#endif

/**
* TRACE prints a message if NDEBUG is not defined.
*/
#ifdef NDEBUG
#define TRACE(...) ((void)0)
#else
#define TRACE(...) Assert::print(__FILE__, __LINE__, __VA_ARGS__)
#endif

/**
* Initializes some log message ring buffers associated to the current thread
* @param name The name of the current thread
* @return Whether the initialization was successful
*/
#define BH_TRACE_INIT(name) ((void)0)

/**
* Adds a message to the log message ring buffer
* @param message The message to be added
*/
#define BH_TRACE_MSG(message) ((void)0)

/**
* Adds the current line of code to the log message ring buffer
*/
#define BH_TRACE ((void)0)
