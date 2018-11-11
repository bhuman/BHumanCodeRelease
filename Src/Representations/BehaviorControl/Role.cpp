/**
 * @file Representations/BehaviorControl/Role.h
 *
 * Implementation of the representation of a robot's behavior role
 *
 * @author Tim Laue, Andreas Stolpmann
 */

#include "Role.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Settings.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"

bool Role::isGoalkeeper() const
{
  return false;
}

void Role::draw() const
{
  DEBUG_DRAWING("representation:Role", "drawingOnField")
  {
    DRAWTEXT("representation:Role", -50, 250, 150, ColorRGBA::white, TypeRegistry::getEnumName(role));
  }

  DEBUG_DRAWING3D("representation:Role3D", "robot")
  {
    static const ColorRGBA colors[numOfRoleTypes] =
    {
      ColorRGBA::black,
      ColorRGBA::black
    };

    int pNumber = Global::getSettings().playerNumber;
    int r(role);
    r = r > 5 ? 5 : r;
    float centerDigit = (pNumber > 1) ? 180.f : 120.0f;
    ROTATE3D("representation:Role3D", 0, 0, pi_2);
    DRAWDIGIT3D("representation:Role3D", r, Vector3f(centerDigit, 0.0f, 500.f), 80, 5, colors[role]);
  }
}
