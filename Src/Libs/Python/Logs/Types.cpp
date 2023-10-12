/**
 * @file Types.cpp
 *
 * This file implements types for representing structured data in a log.
 *
 * @author Arne Hasselbring
 * @author Jan Fiedler
 */

#include "Types.h"

PyTypeVariant Value::toVariant()
{
  if(auto* literal = dynamic_cast<Literal*>(this); literal)
    return literal->operator PyTypeVariant();
  else
    return this;
}

Literal::operator PyTypeVariant()
{
  if(std::holds_alternative<bool>(*this))
    return pybind11::bool_(std::get<bool>(*this));
  else if(std::holds_alternative<long>(*this))
    return pybind11::int_(std::get<long>(*this));
  else if(std::holds_alternative<unsigned long>(*this))
    return pybind11::int_(std::get<unsigned long>(*this));
  else if(std::holds_alternative<double>(*this))
    return pybind11::float_(std::get<double>(*this));
  else if(std::holds_alternative<std::string>(*this))
    return pybind11::str(std::get<std::string>(*this));
  else
  {
    auto& bytes = std::get<std::vector<char>>(*this);
    return pybind11::bytes(bytes.data(), bytes.size());
  }
}

Array::~Array()
{
  for(auto* value : *this)
    delete value;
}

Record::~Record()
{
  for(const auto& pair : attributes)
    delete pair.second;
}

PyTypeVariant Record::getattr(const std::string& name) const
{
  auto it = attributes.find(name);
  if(it == attributes.end())
    throw pybind11::attribute_error("Record has no attribute '" + name + "'");
  return it->second->toVariant();
}

const std::vector<std::string>& Record::getKeys() const
{
  if(keys.empty())
  {
    keys.reserve(attributes.size());
    for(const auto& pair : attributes)
      keys.push_back(pair.first);
  }
  return keys;
}
void TypeStream::select(const char* name, int type, const char*)
{
  Streaming::trimName(name);
  Value** value = stack.empty() ? &root : stack.top().value;
  if(!value)
    stack.emplace(type, nullptr);
  else if(type >= 0)
  {
    // if the DebugDataStreamer didn't always write the size of the array to the stream, the following would be necessary
    /*
    if(!*value)
      *value = new Array;
    */
    auto* array = dynamic_cast<Array*>(*value);
    if(array)
      stack.emplace(type, &((*array)[type]));
    else
      stack.emplace(type, nullptr);
  }
  else
  {
    if(!*value)
      *value = new Record;
    auto* record = dynamic_cast<Record*>(*value);
    if(record)
      stack.emplace(type, &record->attributes[name]);
    else
      stack.emplace(type, nullptr);
  }
}

void TypeStream::deselect()
{
  stack.pop();
}
