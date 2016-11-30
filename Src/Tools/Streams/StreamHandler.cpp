#include "Platform/BHAssert.h"
#include "Tools/Math/Angle.h"
#include "StreamHandler.h"
#include "Streamable.h"
#include <string>
#include <cstring>

StreamHandler::StreamHandler() :
  registering(false),
  registeringBase(false)
{
  basicTypeSpecification[typeid(double).name()];
  basicTypeSpecification[typeid(bool).name()];
  basicTypeSpecification[typeid(float).name()];
  basicTypeSpecification[typeid(short).name()];
  basicTypeSpecification[typeid(unsigned short).name()];
  basicTypeSpecification[typeid(int).name()];
  basicTypeSpecification[typeid(unsigned int).name()];
  basicTypeSpecification[typeid(char).name()];
  basicTypeSpecification[typeid(signed char).name()];
  basicTypeSpecification[typeid(unsigned char).name()];
  basicTypeSpecification[typeid(std::string).name()];
  basicTypeSpecification[typeid(Angle).name()];
}

void StreamHandler::clear()
{
  basicTypeSpecification.clear();
  specification.clear();
  enumSpecification.clear();
  stringTable.clear();
}

void StreamHandler::startRegistration(const char* name, bool registerWithExternalOperator)
{
  if(registeringBase)
  {
    ++registeringEntryStack.top().second.baseClass;
    registeringBase = false;
  }
  else
  {
    Specification::iterator registeringEntry = specification.find(name);
    if(registeringEntry == specification.end())
    {
      specification[name];
      RegisteringAttributes attr;
      attr.registering = true;
      attr.baseClass = 0;
      attr.externalOperator = registerWithExternalOperator;
      registeringEntryStack.push(RegisteringEntry(specification.find(name), attr));
      registering = true;
    }
    else
    {
      RegisteringAttributes attr;
      attr.registering = false;
      attr.baseClass = 0;
      attr.externalOperator = registerWithExternalOperator;
      registeringEntryStack.push(RegisteringEntry(registeringEntry, attr));
      registering = false;
    }
  }
}

void StreamHandler::finishRegistration()
{
  if(registeringEntryStack.size() > 0)
  {
    if(!registeringEntryStack.top().second.baseClass)
      registeringEntryStack.pop();
    else
      --registeringEntryStack.top().second.baseClass;
  }
  if(registeringEntryStack.size() <= 0)
    registering = false;
  else
    registering = registeringEntryStack.top().second.registering;
}

Out& operator<<(Out& out, const StreamHandler& streamHandler)
{
  if(streamHandler.stringTable.empty())
  {
    // basic types
    out << static_cast<unsigned>(streamHandler.basicTypeSpecification.size());
    for(StreamHandler::BasicTypeSpecification::const_iterator basicTypeIter = streamHandler.basicTypeSpecification.begin();
        basicTypeIter != streamHandler.basicTypeSpecification.end(); ++basicTypeIter)
      out << Streaming::demangle(basicTypeIter->first);

    // specification
    out << static_cast<unsigned>(streamHandler.specification.size());
    for(StreamHandler::Specification::const_iterator specificationIter = streamHandler.specification.begin();
        specificationIter != streamHandler.specification.end(); ++specificationIter)
    {
      out << Streaming::demangle(specificationIter->first) << static_cast<unsigned>(specificationIter->second.size());
      for(std::vector<StreamHandler::TypeNamePair>::const_iterator typeNamePairIter = specificationIter->second.begin();
          typeNamePairIter != specificationIter->second.end(); ++typeNamePairIter)
        out << typeNamePairIter->first.c_str() << Streaming::demangle(typeNamePairIter->second);
    }

    // enum specification
    out << static_cast<unsigned>(streamHandler.enumSpecification.size());
    for(StreamHandler::EnumSpecification::const_iterator enumSpecificationIter = streamHandler.enumSpecification.begin();
        enumSpecificationIter != streamHandler.enumSpecification.end(); ++enumSpecificationIter)
    {
      out << Streaming::demangle(enumSpecificationIter->first) << static_cast<unsigned>(enumSpecificationIter->second.size());
      for(std::vector<const char*>::const_iterator enumElementsIter = enumSpecificationIter->second.begin();
          enumElementsIter != enumSpecificationIter->second.end(); ++enumElementsIter)
        out << *enumElementsIter;
    }
  }
  else
  {
    // basic types
    out << static_cast<unsigned>(streamHandler.basicTypeSpecification.size());
    for(StreamHandler::BasicTypeSpecification::const_iterator basicTypeIter = streamHandler.basicTypeSpecification.begin();
        basicTypeIter != streamHandler.basicTypeSpecification.end(); ++basicTypeIter)
      out << basicTypeIter->first;

    // specification
    out << static_cast<unsigned>(streamHandler.specification.size());
    for(StreamHandler::Specification::const_iterator specificationIter = streamHandler.specification.begin();
        specificationIter != streamHandler.specification.end(); ++specificationIter)
    {
      out << specificationIter->first << static_cast<unsigned>(specificationIter->second.size());
      for(std::vector<StreamHandler::TypeNamePair>::const_iterator typeNamePairIter = specificationIter->second.begin();
          typeNamePairIter != specificationIter->second.end(); ++typeNamePairIter)
        out << typeNamePairIter->first.c_str() << typeNamePairIter->second;
    }

    // enum specification
    out << static_cast<unsigned>(streamHandler.enumSpecification.size());
    for(StreamHandler::EnumSpecification::const_iterator enumSpecificationIter = streamHandler.enumSpecification.begin();
        enumSpecificationIter != streamHandler.enumSpecification.end(); ++enumSpecificationIter)
    {
      out << enumSpecificationIter->first << static_cast<unsigned>(enumSpecificationIter->second.size());
      for(std::vector<const char*>::const_iterator enumElementsIter = enumSpecificationIter->second.begin();
          enumElementsIter != enumSpecificationIter->second.end(); ++enumElementsIter)
        out << *enumElementsIter;
    }
  }

  return out;
}

In& operator>>(In& in, StreamHandler& streamHandler)
{
  // note: tables are not cleared, so all data read is appended!
  // However, clear() has to be called once before the first use of this operator

  std::string first,
              second;

  // basic types
  int size;
  in >> size;
  while(size-- > 0)
  {
    in >> first;
    streamHandler.basicTypeSpecification[streamHandler.getString(first)];
  }

  // specification
  in >> size;
  while(size-- > 0)
  {
    int size2;
    in >> first >> size2;
    const char* f = streamHandler.getString(first);
    std::vector<StreamHandler::TypeNamePair> v;
    v.reserve(size2);

    while(size2-- > 0)
    {
      in >> first >> second;
      v.push_back(StreamHandler::TypeNamePair(first, streamHandler.getString(second)));
    }

    streamHandler.specification[f] = v;
  }

  // enum specification
  in >> size;
  while(size-- > 0)
  {
    int size2;
    in >> first >> size2;
    std::vector<const char*> v;
    v.reserve(size2);

    while(size2-- > 0)
    {
      in >> second;
      v.push_back(streamHandler.getString(second));
    }

    streamHandler.enumSpecification[streamHandler.getString(first)] = v;
  }

  return in;
}

const char* StreamHandler::getString(const std::string& string)
{
  StringTable::iterator i;
  i = stringTable.find(string);
  if(i == stringTable.end())
  {
    stringTable[string];
    i = stringTable.find(string);
  }
  return i->first.c_str();
}

void StreamHandler::registerWithSpecification(const char* name, const std::type_info& ti)
{
  if(registering && registeringEntryStack.top().second.registering)
  {
    Streaming::trimName(name);
    std::string nameString = name;
    if(registeringEntryStack.top().second.externalOperator)
    {
      nameString.erase(0, nameString.find(".") + 1);
      registeringEntryStack.top().first->second.push_back(TypeNamePair(nameString, ti.name()));
    }
    else
      registeringEntryStack.top().first->second.push_back(TypeNamePair(name, ti.name()));
  }
}

void StreamHandler::registerEnum(const std::type_info& ti, const char* (*fp)(int))
{
  if(enumSpecification.find(ti.name()) == enumSpecification.end())
  {
    enumSpecification[ti.name()];
    for(int i = 0; (*fp)(i); ++i)
      enumSpecification[ti.name()].push_back((*fp)(i));
  }
}

bool StreamHandler::areSpecificationsForTypesCompatible(StreamHandler& other, std::string type, std::string otherType)
{
  // both StreamHandler contain demangled types
  ASSERT(!stringTable.empty() && !other.stringTable.empty());

  if(type.size() > 8 && type.substr(type.size() - 8) == " __ptr64")
    type = type.substr(0, type.size() - 8);
  if(otherType.size() > 8 && otherType.substr(otherType.size() - 8) == " __ptr64")
    otherType = otherType.substr(0, otherType.size() - 8);

  if(type[type.size() - 1] == ']')
  {
    if(otherType[otherType.size() - 1] != ']')
      return false;

    size_t index = type.size();
    while(type[index - 1] != '[')
      --index;
    size_t index2 = type[index - 2] == ' ' ? index - 2 : index - 1;
    if(type[index2 - 1] == ')')
    {
      index2 -= type[index2 - 2] == '4' ? 8 : 0; // " __ptr64"
      index2 -= type[index2 - 4] == ' ' ? 4 : 3;
    }
    std::string baseType = type.substr(0, index2);
    std::string size = type.substr(index, type.size() - index - 1);

    index = otherType.size();
    while(otherType[index - 1] != '[')
      --index;
    index2 = otherType[index - 2] == ' ' ? index - 2 : index - 1;
    if(otherType[index2 - 1] == ')')
    {
      index2 -= otherType[index2 - 2] == '4' ? 8 : 0; // " __ptr64"
      index2 -= otherType[index2 - 4] == ' ' ? 4 : 3;
    }
    std::string otherBaseType = otherType.substr(0, index2);
    std::string otherSize = otherType.substr(index, otherType.size() - index - 1);

    return size == otherSize && areSpecificationsForTypesCompatible(other, baseType, otherBaseType);
  }
  else if(type[type.size() - 1] == '*')
  {
    std::string baseType = std::string(type).substr(0, type.size() - (type.size() > 1 && type[type.size() - 2] == ' ' ? 2 : 1));
    std::string otherBaseType = std::string(otherType).substr(0, otherType.size() - (otherType.size() > 1 && otherType[otherType.size() - 2] == ' ' ? 2 : 1));
    return areSpecificationsForTypesCompatible(other, baseType, otherBaseType);
  }
  else
  {
    if(type.size() > 6 && type.substr(type.size() - 6) == " const")
      type = type.substr(0, type.size() - 6);
    if(otherType.size() > 6 && otherType.substr(otherType.size() - 6) == " const")
      otherType = otherType.substr(0, otherType.size() - 6);

    const char* t = getString(type.c_str());
    const char* otherT = other.getString(otherType.c_str());
    Specification::const_iterator i = specification.find(t);
    BasicTypeSpecification::const_iterator b = basicTypeSpecification.find(t);
    EnumSpecification::const_iterator e = enumSpecification.find(t);

    if(i != specification.end())
    {
      Specification::const_iterator otherI = other.specification.find(otherT);

      if(otherI != other.specification.end() && i->second.size() == otherI->second.size())
      {
        for(std::vector<StreamHandler::TypeNamePair>::const_iterator j = i->second.begin(), otherJ = otherI->second.begin(); j != i->second.end(); ++j, ++otherJ)
          if(!areSpecificationsForTypesCompatible(other, j->second, otherJ->second))
            return false;
        return true;
      }
    }
    else if(b != basicTypeSpecification.end())
      return type == otherType ||
             (type.find("string") != std::string::npos && otherType.find("string") != std::string::npos) ||
             (type == "float" && otherType.find("Angle") != std::string::npos) || (otherType == "float" && type.find("Angle") != std::string::npos);
    else if(e != enumSpecification.end())
    {
      EnumSpecification::const_iterator otherE = other.enumSpecification.find(otherT);
      if(otherE != other.enumSpecification.end() && e->second.size() >= otherE->second.size())
      {
        for(std::vector<const char*>::const_iterator n = e->second.begin(), otherN = otherE->second.begin(); otherN != otherE->second.end(); ++n, ++otherN)
          if(strcmp(*n, *otherN))
            return false;
        return true;
      }
    }
    else
      ASSERT(false);
  }
  return false;
}
