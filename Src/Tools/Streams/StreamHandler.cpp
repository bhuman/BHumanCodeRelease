
#include "Tools/Streams/StreamHandler.h"
#include "Tools/Streams/Streamable.h"
#include <string>
#include <cstring>

StreamHandler::StreamHandler()
  : registering(false),
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
  basicTypeSpecification[typeid(unsigned char).name()];
  basicTypeSpecification[typeid(std::string).name()];
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
    {
      registeringEntryStack.pop();
    }
    else
    {
      --registeringEntryStack.top().second.baseClass;
    }
  }
  if(registeringEntryStack.size() <= 0)
  {
    registering = false;
  }
  else
  {
    registering = registeringEntryStack.top().second.registering;
  }
}

Out& operator<<(Out& out, const StreamHandler& streamHandler)
{
  // basic types
  out << (unsigned) streamHandler.basicTypeSpecification.size();
  for(StreamHandler::BasicTypeSpecification::const_iterator basicTypeIter = streamHandler.basicTypeSpecification.begin();
      basicTypeIter != streamHandler.basicTypeSpecification.end(); ++basicTypeIter)
    out << Streaming::demangle(basicTypeIter->first);

  // specification
  out << (unsigned) streamHandler.specification.size();
  for(StreamHandler::Specification::const_iterator specificationIter = streamHandler.specification.begin();
      specificationIter != streamHandler.specification.end(); ++specificationIter)
  {
    out << Streaming::demangle(specificationIter->first) << (unsigned) specificationIter->second.size();
    for(std::vector<StreamHandler::TypeNamePair>::const_iterator typeNamePairIter = specificationIter->second.begin();
        typeNamePairIter != specificationIter->second.end(); ++typeNamePairIter)
      out << typeNamePairIter->first.c_str() << Streaming::demangle(typeNamePairIter->second);
  }

  // enum specification
  out << (unsigned) streamHandler.enumSpecification.size();
  for(StreamHandler::EnumSpecification::const_iterator enumSpecificationIter = streamHandler.enumSpecification.begin();
      enumSpecificationIter != streamHandler.enumSpecification.end(); ++enumSpecificationIter)
  {
    out << Streaming::demangle(enumSpecificationIter->first) << (unsigned) enumSpecificationIter->second.size();
    for(std::vector<const char*>::const_iterator enumElementsIter = enumSpecificationIter->second.begin();
        enumElementsIter != enumSpecificationIter->second.end(); ++enumElementsIter)
      out << *enumElementsIter;
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
