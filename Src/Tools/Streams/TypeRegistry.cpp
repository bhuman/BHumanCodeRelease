/**
 * @file TypeRegistry.cpp
 *
 * This file implements a class that stores type information about primitive data types,
 * enumerations, and classes/structures. The avoid a large overhead, the class often
 * uses "const char*" pointers as parameters. It assumes that these stem from a global
 * string pool so that they exist throughout the execution of the program and that
 * each string only exists once, i.e. its address is sufficient to identify it.
 *
 * @author Thomas Röfer
 */

#include <cctype>
#ifndef WINDOWS
#include <cstdlib>
#include <cxxabi.h>
#endif
#include <iostream>
#include <list>
#include <regex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "TypeRegistry.h"
#include "TypeInfo.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/Angle.h"

/** The constants of an enumeration. */
struct Enum
{
  std::vector<std::string> byOrder; /**< In the sequence they are defined. */
  std::unordered_map<std::string, int> byName; /**< Indexed by the constants' names. */
};

/** An attribute of a class. */
struct Attribute
{
  const char* type; /**< The type of the attribute. */
  const char* name; /**< The name of the attribute. */

  /** Keep default constructor. */
  Attribute() = default;

  /**
   * Constructor that initializes all attributes from parameters.
   * @param type The type of the attribute.
   * @param name The name of the attribute.
   */
  Attribute(const char* type, const char* name) : type(type), name(name) {}
};

/** The base class and all attributes of a class. */
struct Class
{
  const char* base; /**< The base class or nullptr if the class has none. */
  std::vector<Attribute> attributes; /**< The list of attribute in the order they are defined. */
};

/** All primitive data types. */
static std::unordered_set<const char*> primitives(
{
  typeid(bool).name(),
  typeid(char).name(),
  typeid(signed char).name(),
  typeid(unsigned char).name(),
  typeid(short).name(),
  typeid(unsigned short).name(),
  typeid(int).name(),
  typeid(unsigned int).name(),
  typeid(float).name(),
  typeid(double).name(),
  typeid(std::string).name(),
  typeid(Angle).name()
});

static std::unordered_map<const char*, Enum> enums; /**< All enumeration types. */
static std::unordered_map<const char*, Class> classes; /**< All classes and structures. */

void TypeRegistry::addEnum(const char* enumeration)
{
  enums[enumeration].byOrder.clear();
}

void TypeRegistry::addEnumConstant(const char* enumeration, const char* name)
{
  Enum& e = enums[enumeration];
  const char* p = name;
  while(std::isalnum(*p) || *p == '_')
    ++p;
  std::string n = std::string(name).substr(0, p - name);
  if(!*p)
  {
    e.byName[n] = static_cast<int>(e.byOrder.size());
    e.byOrder.emplace_back(n);
  }
  else
  {
    ASSERT(!e.byOrder.empty());
    e.byName[n] = static_cast<int>(e.byOrder.size() - 1);
    e.byOrder.back() = n;
  }
}

void TypeRegistry::addClass(const char* theClass, const char* base)
{
  classes[theClass].attributes.clear();
  classes[theClass].base = base;
}

void TypeRegistry::addAttribute(const char* theClass, const char* type, const char* attribute)
{
  const char* p = attribute + std::strlen(attribute);
  while(p > attribute && (std::isalnum(p[-1]) || p[-1] == '_' || p[-1] == '.'))
    --p;
  classes[theClass].attributes.emplace_back(type, p);
}

const char* TypeRegistry::getEnumName(const char* enumeration, int value)
{
  if(*enumeration) // This is a real type name
  {
    auto e = enums.find(enumeration);
    ASSERT(e != enums.end());
    if(value >= 0 && value < static_cast<int>(e->second.byOrder.size()))
      return e->second.byOrder[value].c_str();
  }
  else // This entry was created by the DebugDataStreamer
  {
    const std::vector<std::string>* constants = reinterpret_cast<const std::vector<std::string>* const*>(enumeration)[1];
    if(value >= 0 && value < static_cast<int>(constants->size()))
      return (*constants)[value].c_str();
  }
  return nullptr;
}

int TypeRegistry::getEnumValue(const char* enumeration, const std::string& name)
{
  if(*enumeration) // This is a real type name
  {
    auto e = enums.find(enumeration);
    ASSERT(e != enums.end());
    auto c = e->second.byName.find(name);
    if(c != e->second.byName.end())
      return c->second;
  }
  else // This entry was created by the DebugDataStreamer
  {
    for(int i = 0; getEnumName(enumeration, i); ++i)
      if(getEnumName(enumeration, i) == name)
        return i;
  }
  return -1;
}

#ifdef WINDOWS
static std::regex matchClass("\\bclass ");
static std::regex matchEnum("\\benum ");
static std::regex matchStruct("\\bstruct ");
static std::regex matchUnion("\\bunion ");
static std::regex matchInt64("\\b__int64\\b");
#else
static std::regex matchAnonymousNamespace("::__1\\b");
static std::regex matchCXX11Namespace("::__cxx11\\b");
static std::regex matchNumberSuffix("\\b([0-9][0-9]*)[ul]l*\\b");
static std::regex matchComma(", ");
#endif
static std::regex matchAngularBracket(" >");
static std::regex matchBracket(" \\[");
static std::regex matchAsterisk(" *\\(\\*[^)]*\\)");
static std::regex matchString("std::basic_string<char,std::char_traits<char>,std::allocator<char>>");
static std::regex matchArray("std::array<(.*),([0-9][0-9]*)>");
static std::regex matchList("std::list<(.*),std::allocator<\\1>>");
static std::regex matchVector("std::vector<(.*),(std::allocator|Eigen::aligned_allocator)<\\1>>");

std::string TypeRegistry::demangle(std::string type)
{
#ifdef WINDOWS
  type = std::regex_replace(type, matchClass, "");
  type = std::regex_replace(type, matchEnum, "");
  type = std::regex_replace(type, matchStruct, "");
  type = std::regex_replace(type, matchUnion, "");
  type = std::regex_replace(type, matchInt64, "long long");
#else
  int status;
  size_t length;
  char* buffer = abi::__cxa_demangle(type.c_str(), nullptr, &length, &status);
  if(!buffer)
    return "UNKNOWN";
  else
  {
    type = buffer;
    std::free(buffer);
    type = std::regex_replace(type, matchAnonymousNamespace, "");
    type = std::regex_replace(type, matchCXX11Namespace, "");
    type = std::regex_replace(type, matchNumberSuffix, "$1");
    type = std::regex_replace(type, matchComma, ",");
  }
#endif

  type = std::regex_replace(type, matchAngularBracket, ">");
  type = std::regex_replace(type, matchBracket, "[");
  type = std::regex_replace(type, matchAsterisk, "");
  type = std::regex_replace(type, matchString, "std::string");
  std::string oldType;
  do
  {
    oldType = type;
    type = std::regex_replace(type, matchArray, "$1[$2]");
    type = std::regex_replace(type, matchList, "$1*");
    type = std::regex_replace(type, matchVector, "$1*");
  }
  while(oldType != type);

  return type;
}

void TypeRegistry::print()
{
  for(const std::string& p : primitives)
    std::cout << demangle(p) << std::endl;

  for(const auto& e : enums)
  {
    std::cout << "enum " << demangle(e.first) << " {";
    for(const std::string& c : e.second.byOrder)
      std::cout << (&c == e.second.byOrder.data() ? "" : ", ") << c;
    std::cout << "}" << std::endl;
  }

  for(const auto& c : classes)
  {
    std::cout << "class " << demangle(c.first) << (c.second.base ? " : " + demangle(c.second.base) : "") << " {";
    for(const Attribute& a : c.second.attributes)
      std::cout << (&a == c.second.attributes.data() ? "" : " ") << demangle(a.type) << " " << a.name << ";";
    std::cout << "}" << std::endl;
  }
}

void TypeRegistry::fill(TypeInfo& typeInfo)
{
  for(const char* primitive : primitives)
    typeInfo.primitives.insert(demangle(primitive));

  for(const auto& enumeration : enums)
  {
    std::vector<std::string>& constants = typeInfo.enums[demangle(enumeration.first)];
    constants.reserve(enumeration.second.byOrder.size());
    for(const std::string& constant : enumeration.second.byOrder)
      constants.emplace_back(constant);
  }

  for(const auto& theClass : classes)
  {
    std::vector<TypeInfo::Attribute>& attributes = typeInfo.classes[demangle(theClass.first)];
    std::list<const char*> hierarchy;
    hierarchy.push_front(theClass.first);
    while(classes[hierarchy.front()].base)
      hierarchy.push_front(classes[hierarchy.front()].base);
    for(const auto& entry : hierarchy)
      for(const auto& attribute : classes[entry].attributes)
        attributes.emplace_back(demangle(attribute.type), attribute.name);
  }
}
