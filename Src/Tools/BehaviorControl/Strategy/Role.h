/**
 * @file Role.h
 *
 * This file declares a base class for roles.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "BehaviorBase.h"
#include "Representations/BehaviorControl/SkillRequest.h"

struct Agent;
class Agents;

class Role : public BehaviorBase
{
public:
  enum Type : unsigned char
  {
    none
  };

  static unsigned activeRoleBegin; /**< The index in the enum at which active roles begin. */
  static unsigned positionRoleBegin; /**< The index in the enum at which position roles begin. */

  struct Type_Info
  {
    static Type numOfElements; /**< The number of elements in the enum. */

    /** Registers the enumeration in the type registry. */
    static void reg();
  };

  static bool isActiveRole(Type type)
  {
    return type >= activeRoleBegin && type < positionRoleBegin;
  }

  static bool isPositionRole(Type type)
  {
    return type >= positionRoleBegin && type < Type_Info::numOfElements;
  }

  /**
   * Calculates the role's intention.
   * @param self The agent which executes the role.
   * @param teammates The other agents in the team.
   * @return The skill request that the role wants to be executed.
   */
  virtual SkillRequest execute(const Agent& self, const Agents& teammates) = 0;
};
