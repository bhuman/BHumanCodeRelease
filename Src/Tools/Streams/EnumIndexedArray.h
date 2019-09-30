/**
 * This file defines a macro that allow to declare one-dimensional arrays that use
 * an enumeration type as its index. The enumeration type must have been defined
 * using the ENUM macro. When such an array is streamed and
 * visualized, it appears as a struct with members named after the enumeration
 * constants rather than a sequence of array elements.
 * Examples:
 *   ENUM_INDEXED_ARRAY(int, MessageID) array1;
 *   ENUM_INDEXED_ARRAY(bool, MotionRequest::Motion) array2;
 *   ENUM_INDEXED_ARRAY(MotionRequest::Motion, MessageID) array3;
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Enum.h"

/**
 * Declares a one-dimensional arrays that uses an enumeration type as its index.
 * The array actually declared is derived from std::array<elem, numOf<enum>s>.
 * @param elem The type of the elements of the array.
 * @param enum The enumeration type that is used as the index.
 */
#define ENUM_INDEXED_ARRAY(elem, enum) EnumIndexedArray<elem, enum, enum##_Info>

/**
 * The actual implementation of the enum indexed array.
 * @tparam Elem The type of the array elements.
 * @tparam Enum The enumeration type.
 * @tparam EnumInfo The structure that contains information on the enumeration.
 */
template<typename Elem, typename Enum, typename EnumInfo>
class EnumIndexedArray : public std::array<Elem, EnumInfo::numOfElements>, public Streamable
{
public:
  template<typename... Args>
  EnumIndexedArray(Args&&... args) :
    std::array<Elem, EnumInfo::numOfElements>({{std::forward<Args>(args)...}})
  {}

protected:
  void serialize(In* in, Out* out)
  {
    PUBLISH(EnumInfo::reg);
    PUBLISH(reg);
    for(int i = 0; i < EnumInfo::numOfElements; ++i)
      Streaming::streamIt(in, out, TypeRegistry::getEnumName(static_cast<Enum>(i)), (*this)[i]);
  }

private:
  static void reg()
  {
    REG_CLASS(EnumIndexedArray);
    for(int i = 0; i < EnumInfo::numOfElements; ++i)
      TypeRegistry::addAttribute(_type, typeid(Elem).name(), TypeRegistry::getEnumName(static_cast<Enum>(i)));
  }
};
