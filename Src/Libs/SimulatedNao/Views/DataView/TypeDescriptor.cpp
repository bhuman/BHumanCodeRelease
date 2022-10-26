/**
 * TypeDescriptor.cpp
 *
 *  Created on: Apr 28, 2012
 *      Author: arne
 */

#include "TypeDescriptor.h"
#include <qtvariantproperty.h>

using namespace Type;

/*
 * The following methods return special type ids for meta-types.
 */

int TypeDescriptor::getGroupType()
{
  return QtVariantPropertyManager::groupTypeId();
}

int TypeDescriptor::getEnumTypeId()
{
  return QtVariantPropertyManager::enumTypeId();
}
