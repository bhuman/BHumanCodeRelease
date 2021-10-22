/**
 * @file TurnAngle.cpp
 *
 * This file implements an implementation of the TurnAngle skill
 * that uses the OdometryData to keep track of the orientation
 * (instead of the RobotPose, which means that localization jumps
 * do not affect this skill).
 *
 * @author Arne Hasselbring (the actual behavior is older)
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/Math/Angle.h"
#include <cmath>

SKILL_IMPLEMENTATION(TurnAngleImpl,
{,
  IMPLEMENTS(TurnAngle),
  CALLS(WalkToPoint),
  REQUIRES(OdometryData),
});

class TurnAngleImpl : public TurnAngleImplBase
{
  void execute(const TurnAngle& p) override
  {
    const Pose2f targetRel = Pose2f(Angle::normalize(startRotation + p.angle - theOdometryData.rotation));
    theWalkToPointSkill(targetRel);
  }

  void reset(const TurnAngle&) override
  {
    startRotation = theOdometryData.rotation;
  }

  bool isDone(const TurnAngle& p) const override
  {
    return std::abs(Angle::normalize(startRotation + p.angle - theOdometryData.rotation)) < p.margin;
  }

  Angle startRotation; /**< The global rotation of the robot when the turn started. */
};

MAKE_SKILL_IMPLEMENTATION(TurnAngleImpl);
