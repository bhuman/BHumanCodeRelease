/**
 * @file Role.cpp
 *
 * This file implements a base class for roles.
 *
 * @author Arne Hasselbring
 */

#include "Role.h"
#include "ActiveRole.h"
#include "PositionRole.h"
#include "Streaming/Enum.h"

unsigned Role::activeRoleBegin = 0;
unsigned Role::positionRoleBegin = 0;

Role::Type Role::Type_Info::numOfElements = Role::none;

void Role::Type_Info::reg()
{
  PUBLISH(reg);
  const char* _type = typeid(Type).name();
  TypeRegistry::addEnum(_type);
  TypeRegistry::addEnumConstant(_type, "none");
  unsigned counter = 1;

  activeRoleBegin = counter;
  FOREACH_ENUM(ActiveRole::Type, constant)
  {
    TypeRegistry::addEnumConstant(_type, TypeRegistry::getEnumName(constant));
    ++counter;
  }
  positionRoleBegin = counter;
  FOREACH_ENUM(PositionRole::Type, constant)
  {
    TypeRegistry::addEnumConstant(_type, TypeRegistry::getEnumName(constant));
    ++counter;
  }

  numOfElements = static_cast<Type>(counter);
}
