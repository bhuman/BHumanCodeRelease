/**
 * @file Tools/Debugging/Modify.h
 * Macros make data visible/editable from the RobotConsole.
 *
 * @author Michael Spranger
 * @author Tobias Oberlies
 * @author Thomas Röfer
 */

#include "DebugDataTable.h"

/**
 * Allows for the continuous modification of an object implementing a complete streaming
 * operator with macros from Tools/Streams/Streamable.h (or scalar).
 * @param id An identifier string of the object. Separate hirarchy levels by a single
 *     colon ':', i.e. "cognition:ball percept". The value must be a string constant.
 * @param object The object to be modified.
 */
#define MODIFY(id, object) _MODIFY(id, object, false)

/**
 * Allows for the one-time modification of an object implementing a complete streaming
 * operator with macros from Tools/Streams/Streamable.h (or scalar).
 * @param id An identifier string of the object. Separate hirarchy levels by a single
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
      OUTPUT(idDebugDataResponse, bin, id << Streaming::demangle(typeid(object).name()) << object); \
  } \
  while(false)

/**
 * Allows for the continuous modification of an enumerated value.
 * @param id An identifier string of the variable.
 * The second parameter is the enumeration to be modified.
 * If the type of that enumeration is not defined in the current class,
 * the name of the class in which it is defined has to be specified as
 * third parameter.
 */
#define MODIFY_ENUM(id, ...) _MODIFY_ENUM(id, false, __VA_ARGS__)

/**
 * Allows for the one-time modification of an enumerated value.
 * @param id An identifier string of the variable.
 * The second parameter is the enumeration to be modified.
 * If the type of that enumeration is not defined in the current class,
 * the name of the class in which it is defined has to be specified as
 * third parameter.
 */
#define MODIFY_ENUM_ONCE(id, ...) _MODIFY_ENUM(id, true, __VA_ARGS__)

/** Private helpers for MODIFY_ENUM and MODIFY_ENUM_ONCE. Do not use directly. */
#define _MODIFY_ENUM(id, once, ...) \
  _STREAM_EXPAND(_STREAM_EXPAND(_STREAM_THIRD(__VA_ARGS__, _MODIFY_ENUM_WITH_CLASS, _MODIFY_ENUM_WITHOUT_CLASS))(id, once, __VA_ARGS__))

#define _MODIFY_ENUM_WITHOUT_CLASS(id, once, object) _MODIFY_ENUM2(id, object, once, )
#define _MODIFY_ENUM_WITH_CLASS(id, once, object, class) _MODIFY_ENUM2(id, object, once, class::)
#define _MODIFY_ENUM2(id, object, once, class) \
  do \
  { \
    Streaming::registerEnum(typeid(object), Streaming::castFunction(object, class getName)); \
    Global::getDebugDataTable().updateObject(id, object, once); \
    DEBUG_RESPONSE_ONCE("debug data:" id) \
      OUTPUT(idDebugDataResponse, bin, id << Streaming::demangle(typeid(object).name()) << (unsigned char) object); \
  } \
  while(false)
