/**
 * @file Tools/Debugging/Debugging.h
 *
 * Macros and functions for debugging
 *
 * @author Martin Lötzsch
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/MessageQueue/OutMessage.h"
#ifdef TARGET_TOOL
#include <iostream>
#else
#include "Tools/Debugging/DebugRequest.h"
#endif
#include "Tools/Global.h"

#ifdef TARGET_TOOL
/**
 * A macro for sending warning messages.
 * @param message A message streamable as text.
 */
#define OUTPUT_WARNING(message) \
  do \
    std::cerr << "Warning: " << message << std::endl; \
  while(false)

/**
 * A macro for sending error messages.
 * Note that this macro is enabled regardless of which
 * target is used to build.
 * @param message A message streamable as text.
 */
#define OUTPUT_ERROR(message) \
  do \
    std::cerr << "Error: " << message << std::endl; \
  while(false)

/**
 * Ignore debug requests
 */
#define DECLARED_DEBUG_RESPONSE(id) if(false)
#define DEBUG_RESPONSE(id) if(false)
#define DECLARE_DEBUG_RESPONSE(id) ((void) 0)
#define OUTPUT(type, format, expression) ((void) 0)
#else
 /**
 * A macro for sending debug messages.
 *
 * @param type The type of the message from the MessageID enum in MessageIDs.h
 * @param format The message format of the message (bin or text)
 * @param expression A streamable expression
 *
 * Examples:
 * <pre>
 * OUTPUT(idImage, bin, *pMyImage);
 * OUTPUT_TEXT("MyObject:myFunction() invoked");
 * OUTPUT_TEXT("i: " << i << ", j:" << j);
 * </pre>
 */
#define OUTPUT(type, format, expression) \
  do \
  { \
    Global::getDebugOut().format << expression; \
    Global::getDebugOut().finishMessage(type); \
  } \
  while(false)

 /**
 * Shortcut for outputting text messages.
 * @param expression A streaming expression to output.
 */
#define OUTPUT_TEXT(expression) OUTPUT(idText, text, expression)

/**
 * A macro for sending warning messages.
 * @param message A message streamable as text.
 */
#ifdef NDEBUG
#define OUTPUT_WARNING(message) ((void) 0)
#else
#define OUTPUT_WARNING(message) \
  do \
  { \
    OUTPUT_TEXT("Warning: " << message); \
    OutTextSize _size; \
    _size << "Warning: " << message; \
    char* _buf = new char[_size.getSize() + 1]; \
    OutTextMemory _stream(_buf); \
    _stream << "Warning: " << message; \
    _buf[_size.getSize()] = 0; \
    DebugRequestTable::print(_buf); \
    delete [] _buf; \
  } \
  while(false)
#endif

/**
 * A macro for sending error messages.
 * Note that this macro is enabled regardless of which
 * target is used to build.
 * @param message A message streamable as text.
 */
#define OUTPUT_ERROR(message) \
  do \
  { \
    OUTPUT_TEXT("Error: " << message); \
    OutTextSize _size; \
    _size << "Error: " << message; \
    char* _buf = new char[_size.getSize() + 1]; \
    OutTextMemory _stream(_buf); \
    _stream << "Error: " << message; \
    _buf[_size.getSize()] = 0; \
    DebugRequestTable::print(_buf); \
    delete [] _buf; \
  } \
  while(false)

/**
 * Register debug request if required and check whether it is active.
 * @param id The name of the debug request.
 * @return Is it active?
 */
inline bool _debugRequestActive(const char* id)
{
  if(Global::getDebugRequestTable().poll && Global::getDebugRequestTable().notYetPolled(id))
    OUTPUT(idDebugResponse, text, id << Global::getDebugRequestTable().isActive(id));
  return Global::getDebugRequestTable().isActive(id);
}

/**
 * Declares a debugging switch. This is only necessary in case, where the actual switch
 * is not always reached in each execution cycle.
 * @param id The id of the debugging switch
 */
#define DECLARE_DEBUG_RESPONSE(id) \
  do \
    if(Global::getDebugRequestTable().poll && Global::getDebugRequestTable().notYetPolled(id)) \
      OUTPUT(idDebugResponse, text, id << Global::getDebugRequestTable().isActive(id)); \
  while(false)

/**
 * A debugging switch, allowing the enabling or disabling of the following block.
 * @param id The id of the debugging switch
 */
#define DEBUG_RESPONSE(id) \
  if(_debugRequestActive(id))

/**
 * A debugging switch, allowing the non-recurring execution of the following block.
 * @param id The id of the debugging switch
 */
#define DEBUG_RESPONSE_ONCE(id) \
  if(_debugRequestActive(id) && (Global::getDebugRequestTable().disable(id), true))

/**
 * A debugging switch, allowing the enabling or disabling of the block that follows.
 * @param id The id of the debugging switch
 */
#define DEBUG_RESPONSE_NOT(id) \
  if(!_debugRequestActive(id))

/**
 * Execute following block if debug request is active.
 * The request is not pollable.
 */
#define DECLARED_DEBUG_RESPONSE(id) \
  if(Global::getDebugRequestTable().isActive(id))
#endif // TARGET_TOOL
