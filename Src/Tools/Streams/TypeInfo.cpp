/**
 * @file TypeInfo.cpp
 *
 * This file implements a class that stores type information about primitive data types,
 * enumerations, and classes/structures. In contrast to the class TypeRegistry, this
 * class contains all type information in demangled and unified form. It is also
 * streamable.
 *
 * @author Thomas Röfer
 */

#include "TypeInfo.h"
#include "TypeRegistry.h"
#include <regex>

static const unsigned unifiedTypeNames = 0x80000000;

TypeInfo::TypeInfo(bool fromTypeRegistry)
{
  if(fromTypeRegistry)
    TypeRegistry::fill(*this);
}

bool TypeInfo::areTypesEqual(const TypeInfo& other, const std::string& thisType, const std::string& otherType) const
{
  bool thisIsStaticArray = !thisType.empty() && thisType.back() == ']';
  bool otherIsStaticArray = !otherType.empty() && otherType.back() == ']';
  if(thisIsStaticArray != otherIsStaticArray)
    return false;
  else if(thisIsStaticArray)
  {
    size_t thisEnd = thisType.find_last_of('[');
    size_t otherEnd = otherType.find_last_of('[');
    return thisType.substr(thisEnd) == otherType.substr(otherEnd)
      && areTypesEqual(other, thisType.substr(0, thisEnd), otherType.substr(0, otherEnd));
  }
  else
  {
    bool thisIsDynamicArray = !thisType.empty() && thisType.back() == '*';
    bool otherIsDynamicArray = !otherType.empty() && otherType.back() == '*';
    if(thisIsDynamicArray != otherIsDynamicArray)
      return false;
    else if(thisIsDynamicArray)
      return areTypesEqual(other, thisType.substr(0, thisType.size() - 1), otherType.substr(0, otherType.size() - 1));
    else
    {
      bool thisIsPrimitive = primitives.find(thisType) != primitives.end();
      bool otherIsPrimitive = other.primitives.find(otherType) != other.primitives.end();
      if(thisIsPrimitive != otherIsPrimitive)
        return false;
      else if(thisIsPrimitive)
        return thisType == otherType;
      else
      {
        bool thisIsEnum = enums.find(thisType) != enums.end();
        bool otherIsEnum = other.enums.find(otherType) != other.enums.end();
        if(thisIsEnum != otherIsEnum)
          return false;
        else if(thisIsEnum)
        {
          const std::vector<std::string>& thisConstants = enums.find(thisType)->second;
          const std::vector<std::string>& otherConstants = other.enums.find(otherType)->second;
          return thisConstants == otherConstants
                 || (thisConstants.size() > otherConstants.size()
                     && std::vector<std::string>(thisConstants.begin(), thisConstants.begin() + otherConstants.size()) == otherConstants);
        }
        else
        {
          auto thisClass = classes.find(thisType);
          auto otherClass = other.classes.find(otherType);
          if(thisClass == classes.end() || otherClass == other.classes.end()
             || thisClass->second.size() != otherClass->second.size())
            return false;
          else
          {
            const std::vector<Attribute>& thisAttributes = thisClass->second;
            const std::vector<Attribute>& otherAttributes = otherClass->second;
            for(size_t i = 0; i < thisAttributes.size(); ++i)
              if(!areTypesEqual(other, thisAttributes[i].type, otherAttributes[i].type))
                return false;
            return true;
          }
        }
      }
    }
  }
}

Out& operator<<(Out& out, const TypeInfo& typeInfo)
{
  out << (static_cast<unsigned>(typeInfo.primitives.size()) | unifiedTypeNames);
  for(const std::string& primitive : typeInfo.primitives)
    out << primitive;

  out << static_cast<unsigned>(typeInfo.classes.size());
  for(const std::pair<std::string, std::vector<TypeInfo::Attribute>>& theClass : typeInfo.classes)
  {
    out << theClass.first << static_cast<unsigned>(theClass.second.size());
    for(const TypeInfo::Attribute& attribute : theClass.second)
      out << attribute.name << attribute.type;
  }

  out << static_cast<unsigned>(typeInfo.enums.size());
  for(const std::pair<std::string, std::vector<std::string>>& theEnum : typeInfo.enums)
  {
    out << theEnum.first << static_cast<unsigned>(theEnum.second.size());
    for(const std::string& constant : theEnum.second)
      out << constant;
  }

  return out;
}

static void demangle(std::string& type)
{
  static std::regex matchAnonymousNamespace("::__1\\b");
  static std::regex matchUnsignedLong("([0-9][0-9]*)ul\\b");
  static std::regex matchComma(", ");
  static std::regex matchAngularBracket(" >");
  static std::regex matchBracket(" \\[");
  static std::regex matchAsterisk(" *\\(\\*\\)");
  type = std::regex_replace(type, matchAnonymousNamespace, "");
  type = std::regex_replace(type, matchUnsignedLong, "$1");
  type = std::regex_replace(type, matchComma, ",");
  type = std::regex_replace(type, matchAngularBracket, ">");
  type = std::regex_replace(type, matchBracket, "[");
  type = std::regex_replace(type, matchAsterisk, "");
}

In& operator>>(In& in, TypeInfo& typeInfo)
{
  typeInfo.primitives.clear();
  typeInfo.classes.clear();
  typeInfo.enums.clear();

  std::string type;
  std::string name;
  unsigned size;
  unsigned size2;
  in >> size;
  bool needsTypenameUnification = (size & unifiedTypeNames) == 0;
  size &= ~unifiedTypeNames;
  while(size-- > 0)
  {
    in >> type;
    if(needsTypenameUnification)
      demangle(type);
    typeInfo.primitives.insert(type);
  }

  in >> size;
  while(size-- > 0)
  {
    in >> type >> size2;
    if(needsTypenameUnification)
      demangle(type);
    std::vector<TypeInfo::Attribute>& attributes = typeInfo.classes[type];
    attributes.reserve(size2);
    while(size2-- > 0)
    {
      in >> name >> type;
      if(needsTypenameUnification)
        demangle(type);
      attributes.emplace_back(type, name);
    }
  }

  in >> size;
  while(size-- > 0)
  {
    in >> type >> size2;
    if(needsTypenameUnification)
      demangle(type);
    std::vector<std::string>& constants = typeInfo.enums[type];
    constants.reserve(size2);
    while(size2-- > 0)
    {
      in >> name;
      constants.emplace_back(name);
    }
  }

  return in;
}
