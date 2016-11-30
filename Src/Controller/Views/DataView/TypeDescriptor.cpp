/**
 * TypeDescriptor.cpp
 *
 *  Created on: Apr 28, 2012
 *      Author: arne
 */

#include "TypeDescriptor.h"
#include <string>
#include <qtvariantproperty.h>

using namespace Type;

/*
 * The following template specializations provide a mapping to existing types that qt already
 * supports by default. They only exist to provide a consistent interface.
 * Without those specializations the user would have to use getTypeId for some types
 * and the native QVariant::int or QVariant::double for other types.
 */

template<>
int TypeDescriptor::getTypeId<int>()
{
  return QVariant::Int;
}

template<>
int TypeDescriptor::getTypeId<double>()
{
  return QVariant::Double;
}

template<>
int TypeDescriptor::getTypeId<char>()
{
  return QVariant::Char;
}

template<>
int TypeDescriptor::getTypeId<bool>()
{
  return QVariant::Bool;
}

template<>
int TypeDescriptor::getTypeId<std::string>()
{
  return QVariant::String;
}

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
