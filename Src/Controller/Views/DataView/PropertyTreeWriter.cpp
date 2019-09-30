/**
 * This file implements a helper class to write the property tree to a stream.
 *
 * @author Thomas Röfer
 */

#include "PropertyTreeWriter.h"

void PropertyTreeWriter::inUInt(unsigned int& value)
{
  Entry& e = stack.back();
  if(e.type == -1) // array size
    value = static_cast<unsigned>(e.property->subProperties().size());
  else
    in(value);
}

void PropertyTreeWriter::inAngle(Angle& value)
{
  Entry& e = stack.back();
  value = propertyManager.value(e.property).value<AngleWithUnity>();
}

void PropertyTreeWriter::select(const char* name, int type, const char* enumType)
{
  Entry& e = stack.back();
  const QtProperty* property;
  if(stack.size() == 1 && e.property->subProperties().empty()) // whole tree contains a single property
    property = e.property; // select this only property
  else
    property = e.property->subProperties().at(e.index++); // otherwise enumerate properties
  stack.push_back(Entry(property, type));
}
