/**
 * @file Types.h
 *
 * This file declares types for representing structured data in a log.
 *
 * @author Arne Hasselbring
 * @author Jan Fiedler
 */

#pragma once

#include "Math/Angle.h"
#include "Streaming/InOut.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stack>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

using LiteralVariant = std::variant<bool, long, unsigned long, double, std::string, std::vector<char>>;

class Value;

// For typings instead of `pybind11::object`.
using PyTypeVariant = std::variant<pybind11::bool_, pybind11::int_, pybind11::float_, pybind11::str, pybind11::bytes, Value*>;

/** A base class for all value types for usage in the same STL-containers. */
class Value
{
public:
  virtual ~Value() = default;

  PyTypeVariant toVariant();
};

/** Allow vector modifications to be passed between C++ and Python. Maybe useless here? */
PYBIND11_MAKE_OPAQUE(std::vector<Value*>)

/** Type conversion class for basic types. */
class Literal : public Value, public LiteralVariant
{
public:
  using LiteralVariant::variant;

  operator PyTypeVariant();
};

class Array : public Value, public std::vector<Value*>
{
public:
  using std::vector<Value*>::vector;

  ~Array();
};

/** A collection of named attributes (i.e. a streamable class). */
class Record : public Value
{
public:
  Record() = default;

  Record(Record&& other) :
    attributes(std::move(other.attributes))
  {}

  ~Record();

  /** Returns the attribute with the given name. */
  PyTypeVariant getattr(const std::string& name) const;

  /** Returns all attribute names of this Record. */
  const std::vector<std::string>& getKeys() const;

  std::unordered_map<std::string, Value*> attributes;

private:
  mutable std::vector<std::string> keys;
};

class TypeStream : public Out
{
public:
  TypeStream(Value& value) :
    root(&value)
  {}

private:
  template<typename T>
  void out(const T& value)
  {
    Entry& entry = stack.top();
    if(!entry.value)
      return;
    (*entry.value) = new Literal(value);
  }

  void outBool(bool value) override { out(value); }

  void outChar(char value) override { out(static_cast<long>(value)); }

  void outSChar(signed char value) override { out(static_cast<long>(value)); }

  void outUChar(unsigned char value) override { out(static_cast<unsigned long>(value)); }

  void outShort(short value) override { out(static_cast<long>(value)); }

  void outUShort(unsigned short value) override { out(static_cast<unsigned long>(value)); }

  void outInt(int value) override { out(static_cast<long>(value)); }

  void outUInt(unsigned int value) override
  {
    Entry& entry = stack.top();
    if(!entry.value)
      return;
    if(entry.type == -1)
      (*entry.value) = new Array(value);
    else
      (*entry.value) = new Literal(static_cast<unsigned long>(value));
  }

  void outFloat(float value) override { out(static_cast<double>(value)); }

  void outDouble(double value) override { out(value); }

  void outString(const char* value) override { out(std::string(value)); }

  void outAngle(const Angle& value) override { out(static_cast<double>(value)); }

  void outEndL() override {}

  void write(const void*, std::size_t) override {}

  void select(const char* name, int type, const char* = nullptr) override;

  void deselect() override;

  struct Entry final
  {
    Entry(int type, Value** value) :
      type(type), value(value)
    {}
    int type;
    Value** value;
  };

  Value* root;
  std::stack<Entry, std::vector<Entry>> stack;
};

/** An event annotation of a frame. */
class Annotation
{
public:
  Annotation(const std::string& name, const std::string& description) :
    name(name), description(description)
  {}

  std::string name;
  std::string description;
};
