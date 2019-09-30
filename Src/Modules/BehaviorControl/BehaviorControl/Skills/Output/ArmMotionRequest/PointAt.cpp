/**
 * @file PointAt.cpp
 *
 * This file implements the implementation of the PointAt
 * and PointAtWithArm skills.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/ArmMotionRequest.h"

SKILL_IMPLEMENTATION(PointAtImpl,
{,
  IMPLEMENTS(PointAt),
  IMPLEMENTS(PointAtWithArm),
  REQUIRES(LibCheck),
  MODIFIES(ArmMotionRequest),
});

class PointAtImpl : public PointAtImplBase
{
  void execute(const PointAt& p) override
  {
    if(p.localPoint.y() > threshold)
    {
      setRequest(p.localPoint, Arms::left);
      threshold = -50.f;
    }
    else
    {
      setRequest(p.localPoint, Arms::right);
      threshold = 50.f;
    }
  }

  void reset(const PointAt& p) override
  {
    threshold = 0.f;
  }

  void execute(const PointAtWithArm& p) override
  {
    setRequest(p.localPoint, p.arm);
  }

  void reset(const PointAtWithArm&) override {} // To avoid a stupid clang warning (-Woverloaded-virtual)

  void setRequest(const Vector3f& localPoint, Arms::Arm arm)
  {
    theArmMotionRequest.armMotion[arm] = ArmMotionRequest::pointAt;
    theArmMotionRequest.pointToPointAt = localPoint;
    theLibCheck.setArm(arm);
  }

  float threshold; /**< The current threshold to change the left/right arm decision. */
};

MAKE_SKILL_IMPLEMENTATION(PointAtImpl);
