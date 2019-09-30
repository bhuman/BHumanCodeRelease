/**
 * @file Role.cpp
 *
 * This file implements a representation of a player's role in the 2019 behavior.
 *
 * @author Arne Hasselbring
 */

#include "Role.h"

std::string Role::getName() const
{
  if(isGoalkeeper)
    return "keeper";
  else if(playBall)
    return "striker";
  else
    return "supporter" + std::to_string(supporterIndex);
}

void Role::operator>>(BHumanMessage& m) const
{
  m.theBHumanStandardMessage.isGoalkeeper = isGoalkeeper;
  m.theBHumanStandardMessage.playBall = playBall;
  m.theBHumanStandardMessage.supporterIndex = supporterIndex;
}

void Role::operator<<(const BHumanMessage& m)
{
  if(!m.hasBHumanParts)
    return;

  isGoalkeeper = m.theBHumanStandardMessage.isGoalkeeper;
  playBall = m.theBHumanStandardMessage.playBall;
  supporterIndex = m.theBHumanStandardMessage.supporterIndex;
  numOfActiveSupporters = -1;
}
