/**
 * @file Debugging.h
 *
 * Macros and functions for debugging
 *
 * @author Martin Lötzsch
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/Global.h"
#include "Streaming/Output.h"
#include "Debugging/DebugRequest.h"

#if defined TARGET_ROBOT && defined NDEBUG
/**
 * Ignore debug requests
 */
#define DECLARED_DEBUG_RESPONSE(id) if(false)
#define DEBUG_RESPONSE(id) if(false)
#define DEBUG_RESPONSE_ONCE(id) if(false)
#define DEBUG_RESPONSE_NOT(id) if(true)
#define DECLARE_DEBUG_RESPONSE(id) static_cast<void>(0)
#else
/**
 * Register debug request if required and check whether it is active.
 * @param id The name of the debug request.
 * @return Is it active?
 */
inline bool _debugRequestActive(const char* id)
{
  if(Global::getDebugRequestTable().pollCounter && Global::getDebugRequestTable().notYetPolled(id))
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
    if(Global::getDebugRequestTable().pollCounter && Global::getDebugRequestTable().notYetPolled(id)) \
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
#endif
