/**
 * @file TeammateRoles.cpp
 *
 * This file implements a representation of a team role assignment in the 2019 behavior.
 *
 * @author Arne Hasselbring
 */

#include "TeammateRoles.h"

void TeammateRoles::operator>>(BHumanMessage& m) const
{
  for(size_t i = 0; i < BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i)
  {
    m.theBHumanStandardMessage.teammateRolesIsGoalkeeper[i] = false;
    m.theBHumanStandardMessage.teammateRolesPlayBall[i] = false;
    m.theBHumanStandardMessage.teammateRolesPlayerIndex[i] = (*this)[i + 1];
  }
  m.theBHumanStandardMessage.teammateRolesTimestamp = timestamp;
  m.theBHumanStandardMessage.captain = captain;
}

void TeammateRoles::operator<<(const BHumanMessage& m)
{
  if(!m.hasBHumanParts)
    return;

  for(size_t i = 0; i < BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i)
  {
    (*this)[i + 1] = m.theBHumanStandardMessage.teammateRolesPlayerIndex[i];
  }
  timestamp = m.toLocalTimestamp(m.theBHumanStandardMessage.teammateRolesTimestamp);
  captain = m.theBHumanStandardMessage.captain;
}

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
