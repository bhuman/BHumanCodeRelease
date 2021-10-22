/**
 * @file TurnToPoint.cpp
 *
 * This file implements an implementation of the TurnToPoint skill.
 *
 * @author Nicole Schrader
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/Math/Angle.h"
#include <cmath>

SKILL_IMPLEMENTATION(TurnToPointImpl,
{,
  IMPLEMENTS(TurnToPoint),
  CALLS(TurnAngle),
  REQUIRES(OdometryData),
});

class TurnToPointImpl : public TurnToPointImplBase
{
  void execute(const TurnToPoint&) override
  {
    theTurnAngleSkill(rotation);
  }

  void reset(const TurnToPoint& p) override
  {
    rotation = p.target.angle();
  }

  bool isDone(const TurnToPoint& p) const override
  {
    return std::abs(Angle::normalize(rotation - theOdometryData.rotation)) < p.margin;
  }

  Angle rotation; /**< The rotation that the robot should turn. */
};

MAKE_SKILL_IMPLEMENTATION(TurnToPointImpl);
