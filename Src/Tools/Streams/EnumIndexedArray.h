/**
 * This file defines a macro that allow to declare one-dimensional arrays that use
 * an enumeration type as its index. The enumeration type must have been defined
 * using the ENUM or GLOBAL_ENUM macros. When such an array is streamed and 
 * visualized, it appears as a struct with members named after the enumeration
 * constants rather than a sequence of array elements.
 * Examples:
 *   ENUM_INDEXED_ARRAY(int, MessageID) array1;
 *   ENUM_INDEXED_ARRAY(bool, (MotionRequest) Motion) array2;
 *   ENUM_INDEXED_ARRAY((MotionRequest) Motion, MessageID) array3;
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Enum.h"

/**
 * Declares a one-dimensional arrays that uses an enumeration type as its index.
 * The array actually declared is derived from std::array<elem, numOf<enum>s>.
 * @param elem The type of the elements of the array. If this is also an enumeration
 *             type, it must be defined as "(enum-domain) type", where "enum-domain"
 *             is the class or namespace in which the enumeration type was defined,
 *             i.e. "enum-domain::type" would be the fully qualified type. Note
 *             that this is even necessary if the enumeration type is defined in the
 *             current class.
 * @param enum The enumeration type that is used as the index. If the type is not
 *             declared in the current class, it must also be specified as
 *             "(enum-domain) type".
 */
#define ENUM_INDEXED_ARRAY(elem, enum) \
  EnumIndexedArray<_ENUM_TYPE(elem), _ENUM_GETNAME(elem), _ENUM_TYPE(enum), _ENUM_DOMAIN(enum)_FOREACH_ENUM_NUMOFS(_FOREACH_ENUM_TYPE(enum)), _ENUM_DOMAIN(enum)getName>

/**
 * Return the fully qualified getName function. If the type passed is
 * not an enum, nullptr is returned.
 * @param elem The element type. If it is of the form "(enum-domain) type", it is
 *             assumed to be an enumeration type. Otherwise, a normal type is assumed.
 */
#if defined _MSC_VER && !defined Q_MOC_RUN
#define _ENUM_GETNAME(elem) _ENUM_GETNAME_I(elem)*##/
#define _ENUM_GETNAME_I(...) _STREAM_JOIN(_ENUM_GETNAME_II_, _STREAM_SEQ_SIZE(__VA_ARGS__))(__VA_ARGS__)
#define _ENUM_GETNAME_II_0(...) nullptr/##*
#define _ENUM_GETNAME_II_1(...) _ENUM_GETNAME_III __VA_ARGS__
#define _ENUM_GETNAME_III(...) __VA_ARGS__::getName /##*
#else
#define _ENUM_GETNAME(...) _ENUM_GETNAME_I(__VA_ARGS__))
#define _ENUM_GETNAME_I(...) _STREAM_JOIN(_ENUM_GETNAME_II_, _STREAM_SEQ_SIZE(__VA_ARGS__))(__VA_ARGS__)
#define _ENUM_GETNAME_II_0(...) nullptr _STREAM_DROP(
#define _ENUM_GETNAME_II_1(...) _ENUM_GETNAME_III __VA_ARGS__
#define _ENUM_GETNAME_III(...) __VA_ARGS__::getName _STREAM_DROP(
#endif

/**
 * Return the domain of an enumeration type.
 * @param enum The enum type. If it is of the form "(enum-domain) type", "enum-domain::"
 *             is returned. Otherwise, nothing is returned.
 */
#if defined _MSC_VER && !defined Q_MOC_RUN
#define _ENUM_DOMAIN(enum) _ENUM_DOMAIN_I(enum)*##/
#define _ENUM_DOMAIN_I(...) _STREAM_JOIN(_ENUM_DOMAIN_II_, _STREAM_SEQ_SIZE(__VA_ARGS__))(__VA_ARGS__)
#define _ENUM_DOMAIN_II_0(...) /##*
#define _ENUM_DOMAIN_II_1(...) _ENUM_DOMAIN_III __VA_ARGS__
#define _ENUM_DOMAIN_III(...) __VA_ARGS__:: /##*
#else
#define _ENUM_DOMAIN(enum) _ENUM_DOMAIN_I(enum))
#define _ENUM_DOMAIN_I(...) _STREAM_JOIN(_ENUM_DOMAIN_II_, _STREAM_SEQ_SIZE(__VA_ARGS__))(__VA_ARGS__)
#define _ENUM_DOMAIN_II_0(...) _STREAM_DROP(
#define _ENUM_DOMAIN_II_1(...) _ENUM_DOMAIN_III __VA_ARGS__
#define _ENUM_DOMAIN_III(...) __VA_ARGS__:: _STREAM_DROP(
#endif

/**
 * Return the type part of an enumeration type.
 * @param enum The enum type. A possible prefix of "(enum-domain)" is simply removed.
 */
#define _ENUM_TYPE(...) _ENUM_TYPE_I(__VA_ARGS__)
#define _ENUM_TYPE_I(...) _STREAM_JOIN(_ENUM_TYPE_II_, _STREAM_SEQ_SIZE(__VA_ARGS__))(__VA_ARGS__)
#define _ENUM_TYPE_II_0(...) __VA_ARGS__
#define _ENUM_TYPE_II_1(...) _ENUM_TYPE_III __VA_ARGS__
#define _ENUM_TYPE_III(...) __VA_ARGS__::

/**
 * The actual implementation of the enum indexed array.
 * @param Elem The type of the array elements.
 * @param getElemName A function that delivers the names of enum constants if
 *                    Elem is an enum. Otherwise, nullptr must be passed.
 * @param Enum The enumeration type.
 * @param numOfElements The number of elements of the enumeration type and thereby
 *                      the number of elements of the array declared.
 * @param getEnumName A function that delivers the names of constants of Enum.
 */
template <typename Elem, const char* (*getElemName)(Elem), typename Enum, Enum numOfElements, const char* (*getEnumName)(Enum)>
class EnumIndexedArray : public std::array<Elem, numOfElements>, public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN
    for(int i = 0; i < numOfElements; ++i)
      Streaming::streamIt(in, out, getEnumName(static_cast<Enum>(i)), (*this)[i], reinterpret_cast<const char* (*)(int)>(getElemName));
    STREAM_REGISTER_FINISH
  }
};
