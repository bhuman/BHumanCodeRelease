/**
 * @file InExpr.cpp
 *
 * This file implements a stream for configuration files that allows
 * arithmetic expressions (including predefined symbols) as values
 * (currently only for floats).
 *
 * @author Arne Hasselbring
 */

#include "InExpr.h"
#include "Math/BHMath.h"
#ifdef TARGET_ROBOT
#include "Platform/BHAssert.h"
#define ERROR(...) FAIL(__VA_ARGS__)
#else
#include "Streaming/Output.h"
#define ERROR(...) OUTPUT_ERROR(__VA_ARGS__)
#endif
#include <cctype>

void InExprMemory::inFloat(float& value)
{
  std::string str, buf;
  while(!eof())
  {
    inString(buf);
    str += buf;
  }

  const char* ptr = str.c_str();
  value = readExpr(ptr);
  while(std::isspace(*ptr))
    ++ptr;
  if(*ptr != '\0')
    ERROR("Value could not be parsed: " << str);
}

float InExprMemory::readExpr(const char*& ptr)
{
  float expr = readTerm(ptr);
  while(*ptr == '+' || *ptr == '-')
  {
    const bool plus = *ptr == '+';
    const float term = readTerm(++ptr);
    expr = plus ? (expr + term) : (expr - term);
  }
  return expr;
}

float InExprMemory::readTerm(const char*& ptr)
{
  float term = readFactor(ptr);
  while(*ptr == '*' || *ptr == '/')
  {
    const bool times = *ptr == '*';
    const float factor = readFactor(++ptr);
    term = times ? (term * factor) : (term / factor);
  }
  return term;
}

float InExprMemory::readFactor(const char*& ptr)
{
  while(std::isspace(*ptr))
    ++ptr;
  if(*ptr == '-')
    return -readFactor(++ptr);
  return readLiteral(ptr);
}

float InExprMemory::readLiteral(const char*& ptr)
{
  if(*ptr == '(')
  {
    const float bracketedExpr = readExpr(++ptr);
    if(*ptr != ')')
      ERROR("No closing parenthesis: " << ptr);
    else
      ++ptr;
    while(std::isspace(*ptr))
      ++ptr;
    return bracketedExpr;
  }
  else if(*ptr == '.' || (*ptr >= '0' && *ptr <= '9'))
  {
    char* end = nullptr;
    const float literal = std::strtof(ptr, &end);
    ptr = end;
    while(std::isspace(*ptr))
      ++ptr;
    return literal;
  }
  else if((*ptr >= 'A' && *ptr <= 'Z') || (*ptr >= 'a' && *ptr <= 'z'))
  {
    const char* end = ptr + 1;
    while((*end >= 'A' && *end <= 'Z') || (*end >= 'a' && *end <= 'z') || (*end >= '0' && *end <= '9'))
      ++end;
    const std::string key(ptr, end - ptr);
    ptr = end;
    while(std::isspace(*ptr))
      ++ptr;
    const auto it = symbols.find(key);
    if(it != symbols.end())
      return it->second;
    else
      ERROR("Symbol does not exist: " << key);
    return 0.f;
  }
  else
  {
    ERROR("Literal could not be parsed: " << ptr);
    return 0.f;
  }
}

InExprMapFile::InExprMapFile(const std::string& name, const std::unordered_map<std::string, float>& symbols, unsigned errorMask) :
  InMapFile(name, errorMask),
  symbols(symbols)
{}

In* InExprMapFile::createLiteralStream(const std::string& literal)
{
  return new InExprMemory(literal.c_str(), literal.length(), symbols);
}
