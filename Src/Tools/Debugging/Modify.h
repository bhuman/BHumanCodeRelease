/**
 * @file Tools/Debugging/Modify.h
 * Macros make data visible/editable from the RobotConsole.
 *
 * @author Michael Spranger
 * @author Tobias Oberlies
 * @author Thomas Röfer
 */

#include "DebugDataTable.h"

#if !defined TARGET_ROBOT || !defined NDEBUG

/**
 * Allows for the continuous modification of an object implementing a complete streaming
 * operator with macros from Tools/Streams/Streamable.h (or scalar).
 * @param id An identifier string of the object. Separate hierarchy levels by a single
 *     colon ':', i.e. "cognition:ball percept". The value must be a string constant.
 * @param object The object to be modified.
 */
#define MODIFY(id, object) _MODIFY(id, object, false)

/**
 * Allows for the one-time modification of an object implementing a complete streaming
 * operator with macros from Tools/Streams/Streamable.h (or scalar).
 * @param id An identifier string of the object. Separate hierarchy levels by a single
 *     colon ':', i.e. "cognition:ball percept". The value must be a string constant.
 * @param object The object to be modified.
 */
#define MODIFY_ONCE(id, object) _MODIFY(id, object, true)

/** Private helper for MODIFY and MODIFY_ONCE. Do not use directly. */
#define _MODIFY(id, object, once) \
  do \
  { \
    Global::getDebugDataTable().updateObject(id, object, once); \
    DEBUG_RESPONSE_ONCE("debug data:" id) \
      OUTPUT(idDebugDataResponse, bin, id << TypeRegistry::demangle(typeid(object).name()) << object); \
  } \
  while(false)

#else

#define MODIFY(id, object) static_cast<void>(0)
#define MODIFY_ONCE(id, object) static_cast<void>(0)

#endif
