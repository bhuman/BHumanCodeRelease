/**
 * PropertyManager.cpp
 *
 *  Created on: Apr 27, 2012
 *      Author: arne
 */

#include "PropertyManager.h"

PropertyManager::PropertyManager()
{
  initTypes();

  connect(this, SIGNAL(valueChanged(QtProperty*, const QVariant&)),
          this, SLOT(slotValueChanged(QtProperty*, const QVariant&)));
}

PropertyManager::~PropertyManager()
{
  QList<TypeDescriptor*> typeDescriptors(theDescriptors.values());

  for(TypeDescriptor* typeDescriptor : typeDescriptors)
    delete typeDescriptor;

  theDescriptors.clear();
}

bool PropertyManager::isPropertyTypeSupported(int type) const
{
  if(theDescriptors.contains(type))
    return true;
  else
    return QtVariantPropertyManager::isPropertyTypeSupported(type);
}

QVariant PropertyManager::value(const QtProperty* pProperty) const
{
  if(theValues.contains(pProperty))
  {
    //this is a property with custom type and therefore managed by us
    return theValues[pProperty];
  }
  else
  {
    //This is a property managed by our base.
    return QtVariantPropertyManager::value(pProperty);
  }
}

int PropertyManager::valueType(int propertyType) const
{
  if(theDescriptors.contains(propertyType))
    return propertyType;
  else
    return QtVariantPropertyManager::valueType(propertyType);
}

QString PropertyManager::valueText(const QtProperty* pProperty) const
{
  int propType = propertyType(pProperty);
  if(theValues.contains(pProperty) && theDescriptors.contains(propType))
  {
    QVariant var = theValues[pProperty];
    return theDescriptors[propType]->toString(var);
  }
  else
    return QtVariantPropertyManager::valueText(pProperty);
}

void PropertyManager::initTypes()
{
  theDescriptors[UIntDescriptor().getSupportedTypeId()] = new UIntDescriptor();
  theDescriptors[UCharDescriptor().getSupportedTypeId()] = new UCharDescriptor();
  theDescriptors[UShortDescriptor().getSupportedTypeId()] = new UShortDescriptor();
  theDescriptors[FloatDescriptor().getSupportedTypeId()] = new FloatDescriptor();
  theDescriptors[ShortDescriptor().getSupportedTypeId()] = new ShortDescriptor();
  theDescriptors[AngleTypeDescriptor().getSupportedTypeId()] = new AngleTypeDescriptor();
}

void PropertyManager::setValue(QtProperty* pProperty, const QVariant& val)
{
  if(theValues.contains(pProperty))
  {
    if(theValues[pProperty] != val)
    {
      theValues[pProperty] = val;
      emit propertyChanged(pProperty);
      emit valueChanged(pProperty, val);
    }
  }
  else
    QtVariantPropertyManager::setValue(pProperty, val);
}

void PropertyManager::initializeProperty(QtProperty* pProperty)
{
  int typeId = propertyType(pProperty);

  if(theDescriptors.contains(typeId))
    theValues[pProperty] = theDescriptors[typeId]->createDefault();

  QtVariantPropertyManager::initializeProperty(pProperty);
}

void PropertyManager::uninitializeProperty(QtProperty* pProperty)
{
  if(theDescriptors.contains(propertyType(pProperty)))
    theValues.remove(pProperty);
  QtVariantPropertyManager::uninitializeProperty(pProperty);
}

void PropertyManager::slotValueChanged(QtProperty* pProperty, const QVariant& value)
{
  //update values
  if(theValues.contains(pProperty))
    theValues[pProperty] = value;
}
