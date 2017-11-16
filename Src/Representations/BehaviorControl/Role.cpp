/**
 * @file Representations/BehaviorControl/Role.h
 *
 * Implementation of the representation of a robot's behavior role
 *
 * @author Tim Laue, Andreas Stolpmann
 */

#include "Role.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Settings.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Blackboard.h"

B_HULKs::Role Role::toBHulksRole(const Role::RoleType type)
{
  switch(type)
  {
    case Role::keeper:
    case Role::attackingKeeper:
    case Role::penaltyKeeper:
      return B_HULKs::Role::King;
    case Role::striker:
    case Role::penaltyStriker:
      return B_HULKs::Role::Queen;
    case Role::defender:
      return B_HULKs::Role::Rook;
    case Role::supporter:
      return B_HULKs::Role::Knight;
    case Role::bishop:
      return B_HULKs::Role::Bishop;
    case Role::none:
    case Role::undefined:
    case Role::numOfRoleTypes:
    default:
      return B_HULKs::Role::beatenPieces;
  }
}

Role::RoleType Role::fromBHulksRole(const B_HULKs::Role role)
{
  switch(role)
  {
    case B_HULKs::Role::King:
      return Role::keeper;
    case B_HULKs::Role::Rook:
      return Role::defender;
    case B_HULKs::Role::Queen:
      return Role::striker;
    case B_HULKs::Role::Knight:
      return Role::supporter;
    case B_HULKs::Role::Bishop:
      return Role::bishop;
    case B_HULKs::Role::beatenPieces:
    default:
      return Role::undefined;
  }
}

bool Role::isGoalkeeper() const
{
  const bool goalKeeper = role == Role::keeper
                          || role == Role::attackingKeeper
                          || role == Role::penaltyKeeper;
#ifndef NDEBUG
  if(goalKeeper
     && Blackboard::getInstance().exists("RobotInfo")
     && Blackboard::getInstance().exists("GameInfo")
     && Blackboard::getInstance().exists("CognitionStateChanges"))
  {
    auto robotNumber = static_cast<RobotInfo&>(Blackboard::getInstance()["RobotInfo"]).number;
    auto secondaryState = static_cast<GameInfo&>(Blackboard::getInstance()["GameInfo"]).secondaryState;
    auto lastSecondaryState = static_cast<CognitionStateChanges&>(Blackboard::getInstance()["CognitionStateChanges"]).lastSecondaryGameState;
    if(secondaryState == lastSecondaryState)
    {
      ASSERT(robotNumber == 1 || secondaryState == STATE2_PENALTYSHOOT);
    }
  }
#endif
  return goalKeeper;
}

void Role::draw() const
{
  DEBUG_DRAWING("representation:Role", "drawingOnField")
  {
    DRAWTEXT("representation:Role", -50, 250, 150, ColorRGBA::white, getName(role));
  }

  DEBUG_DRAWING3D("representation:Role3D", "robot")
  {
    static const ColorRGBA colors[numOfRoleTypes] =
    {
      ColorRGBA::black,
      ColorRGBA::blue,
      ColorRGBA::blue,
      ColorRGBA::red,
      ColorRGBA::white,
      ColorRGBA::green,
      ColorRGBA::red,
      ColorRGBA::blue,
      ColorRGBA::black
    };

    int pNumber = Global::getSettings().playerNumber;
    int r(role);
    r = r > 8 ? 8 : r;
    float centerDigit = (pNumber > 1) ? 180.f : 120.0f;
    ROTATE3D("representation:Role3D", 0, 0, pi_2);
    DRAWDIGIT3D("representation:Role3D", r, Vector3f(centerDigit, 0.0f, 500.f), 80, 5, colors[role]);
  }
}

void TeammateRoles::operator >> (BHumanMessage& m) const
{
  for(size_t i = 0; i < sizeof(m.theBHULKsStandardMessage.roleAssignments); ++i)
    m.theBHULKsStandardMessage.roleAssignments[i] = Role::toBHulksRole((*this)[i + 1]);
}

void TeammateRoles::operator<< (const BHumanMessage& m)
{
  for(size_t i = 0; i < sizeof(m.theBHULKsStandardMessage.roleAssignments); ++i)
    if(m.theBHULKsStandardMessage.roleAssignments[i] != B_HULKs::Role::beatenPieces)
      (*this)[i + 1] = Role::fromBHulksRole(m.theBHULKsStandardMessage.roleAssignments[i]);
}

Role::RoleType& TeammateRoles::operator[](const size_t i)
{
  while(roles.size() <= i)
    roles.push_back(Role::RoleType::none);
  return roles[i];
}

Role::RoleType TeammateRoles::operator[](const size_t i) const
{
  if(roles.size() <= i)
    return Role::RoleType::none;
  return roles[i];
}

const char* TeammateRoles::getName(Role::RoleType e)
{
  return Role::getName(e);
}
