/**
 * @file TeammateRoles.cpp
 *
 * This file implements a representation of a team role assignment in the 2019 behavior.
 *
 * @author Arne Hasselbring
 */

#include "TeammateRoles.h"

int& TeammateRoles::operator[](const size_t i)
{
  while(roles.size() <= i)
    roles.push_back(-1);
  return roles[i];
}

int TeammateRoles::operator[](const size_t i) const
{
  if(roles.size() <= i)
    return -1;
  return roles[i];
}
