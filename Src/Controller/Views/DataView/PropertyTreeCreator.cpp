/**
 * The file implements a helper class to create and update the property tree.
 *
 * @author Thomas Röfer
 */

#include "PropertyTreeCreator.h"
#include "Tools/Motion/SensorData.h"
#include "Tools/Streams/TypeRegistry.h"

void PropertyTreeCreator::outUChar(unsigned char value)
{
  Entry& e = stack.back();
  if(e.enumType)
  {
    e.property = view.getProperty(e.path, TypeDescriptor::getEnumTypeId(), e.name.c_str(), e.parent);
    QStringList enumNames = e.property->attributeValue("enumNames").value<QStringList>();
    if(enumNames.empty())
    {
      for(int i = 0; TypeRegistry::getEnumName(e.enumType, i); ++i)
        enumNames << TypeRegistry::getEnumName(e.enumType, i);
      e.property->setAttribute("enumNames", enumNames);
    }
    e.property->setValue(value);
  }
  else
    out(value);
}

void PropertyTreeCreator::outUInt(unsigned int value)
{
  Entry& e = stack.back();
  if(e.type != -1)
    out(value);
  else
  {
    e.property = view.getProperty(e.path, TypeDescriptor::getGroupType(), e.name.c_str(), e.parent);
    while(value < static_cast<unsigned>(e.property->subProperties().size()))
      e.property->removeSubProperty(e.property->subProperties().last());
  }
}

void PropertyTreeCreator::outAngle(const Angle& value)
{
  Entry& e = stack.back();
  ASSERT(!e.property);
  e.property = view.getProperty(e.path, TypeDescriptor::getTypeId<AngleWithUnity>(), e.name.c_str(), e.parent);
  AngleWithUnity angle = value;
  angle.deg = e.property->value().value<AngleWithUnity>().deg;
  e.property->setValue(QVariant::fromValue(angle));
}

void PropertyTreeCreator::select(const char* name, int type, const char* enumType)
{
  QtVariantProperty* parent = 0;
  std::string path;

  if(!stack.empty())
  {
    ASSERT(name || type >= 0);
    Entry& e = stack.back();
    if(!e.property) // must be a record or static array -> create it
      e.property = view.getProperty(e.path, TypeDescriptor::getGroupType(), e.name.c_str(), e.parent);
    if(type < 0) // delay adding to parent for list elements to circumvent a bug that adds everything twice
      parent = e.property;
    path = e.path;
  }

  stack.push_back(Entry(type, enumType, parent, path, name));
}

void PropertyTreeCreator::deselect()
{
  QtVariantProperty* property = stack.back().property;
  ASSERT(property);
  stack.pop_back();
  if(!stack.empty())
    stack.back().property->addSubProperty(property);
  else
    root = property;
}
